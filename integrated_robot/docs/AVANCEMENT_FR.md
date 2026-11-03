# Avancement du projet (mis à jour le 2 novembre 2026)

## Où on en est en une phrase

Le robot trie les objets sur le vrai UR5 (reaching → saisie → dépôt dans le bon bac → retour à la
position d'observation) et **tous les tests ont marché** ; il reste à **implémenter la partie
automatique du pipeline** (caméra → reconnaissance → prise → dépôt, enchaînés sans intervention).

---

## Ce qui est fait

### Avant la séance du 2 novembre (dans le repo depuis le 30 octobre)

- **Planification de mouvement** avec des algorithmes publiés : cinématique inverse UR5 analytique
  (Hawkins 2013), mouvement articulaire direct sinon RRT-Connect (Kuffner & LaValle 2000) +
  raccourcissement, vérification de collision par capsules, profil trapézoïdal.
- **Machine à états** du cycle : reaching (`TRANSFER_TO_PICK`) → picking (ouvrir, descendre,
  fermer, lever) → transport → dépôt → retrait. Tout le cycle est planifié et vérifié **avant** le
  premier mouvement ; après un échec, le programme s'arrête et ne fait rien tout seul.
- **Lien avec la vision (option A)** : YOLO dit *quel* objet est dans la zone de prise (seul,
  stable 5 images, confiance ≥ 0,60) ; le robot rejoue la prise apprise de cette classe. Le bac
  vient de `configs/sorting_policy.json`.
- **Option B préparée mais désactivée** : décaler la prise jusqu'à la position mesurée par la
  caméra. Refusée tant qu'il n'y a pas d'étalonnage caméra → robot validé.
- **Adaptateur robot réel** (`ur_rtde`) avec verrous : TCP vérifié au démarrage, une commande à la
  fois, mode pas-à-pas forcé tant que la cellule n'est pas mesurée.

### Séance au labo du 2 novembre

**Connexion**
- PC relié au robot par l'adaptateur USB → RJ45 (carte « Ethernet 4 »). Pas de DHCP sur ce câble :
  adresse fixe `147.250.35.200`, masque `255.255.255.0`, sans passerelle
  (`New-NetIPAddress` en PowerShell administrateur). `ping 147.250.35.40` OK.
- `ur_rtde` 1.6.5 installé sur le laptop.
- Bras mis sous tension depuis la tablette : *Setup Robot → Initialize Robot → ON → START*.

**Vérification en lecture seule** (`tools/check_robot.py`)
- Robot en mode 7 (RUNNING), notre modèle et le contrôleur d'accord à **2,2 mm**, modèle de
  collision valide.
- Cela confirme que **le TCP du pendant est à 0** (question ouverte réglée).

**Premiers mouvements réels**
- Essai à blanc (planificateur sur robot simulé depuis la position réelle du bras), puis cycles
  réels en pas-à-pas avec la bouteille : **le cycle complet a marché**.
- Un essai est allé « au mauvais endroit » : le journal montre des commandes **identiques** au
  1/1000 rad près à tous les essais ; c'est la bouteille qui avait été reposée ailleurs
  (placement fixe).

**Pince OnRobot RG6** (mesures par enregistrement RTDE de la prise outil)
- Branchée sur la prise du poignet, 24 V. **Sortie outil 0 ON = fermer, OFF = ouvrir** : confirmé.
- Signaux observés sur 3 saisies : entrée outil 1 = 0 pendant le mouvement, 1 quand la pince est
  prête ; entrée outil 0 = 1 quand un objet est saisi ; entrée analogique 0 ≈ 10 V ouverte,
  ≈ 6,5 V en serrant.
- **Une fermeture ou une ouverture dure ≈ 4,1 s.** Le programme n'attendait que 0,8 s : en mode
  automatique le bras serait parti avant la fin du mouvement des doigts. **Corrigé : 5,0 s**
  (commit `d0cfe59`).
- Panne observée : la pince recevait l'ordre mais le moteur ne tournait plus. **Couper puis remettre
  la tension outil** (onglet I/O, 0 V puis 24 V) l'a débloquée. Cause probable : l'arrêt de
  sécurité de la pince après un choc (non prouvé).

**Prise inclinée et nouvelles prises**
- Étude par le calcul : on peut incliner la pince jusqu'à **≈ 40° sous l'horizontale** sans changer
  la posture du bras ; au-delà le bras devrait se retourner ; la prise verticale est hors
  d'atteinte (table trop haute par rapport au pied du robot).
- Nouvel outil **`tools/teach_grasp.py`** (lecture seule) : on place la pince à la main sur la
  prise, l'outil calcule l'approche (recul de 8 cm dans l'axe de la pince) et la levée (verticale,
  ou recul dans l'axe si la verticale est hors d'atteinte) et vérifie limites, collisions et
  trajets droits. Activé par objet dans `taught_route.grasp_overrides` ; le paquet de l'équipe
  vision n'est jamais modifié. 9 tests ajoutés (98 au total).
- **Prises apprises sur place** (commit `a9a886b`) :

  | Objet | Prise |
  |---|---|
  | éponge | inclinée 28°, levée dans l'axe de la pince |
  | gobelet | 5,5°, levée verticale |
  | bouteille et canette | même case, même posture, 2,4°, levée verticale |

  Les emplacements des objets ont changé par rapport aux prises de l'équipe (jusqu'à 14 cm).

**Tests des nouvelles prises sur le vrai robot (après-midi, pas-à-pas, `--replay`)**
- **Gobelet et bouteille : cycle complet réussi.** Le gobelet se teste désormais sans caméra grâce à
  `fixtures/vision/paper_cup_stable.jsonl`.
- **Éponge : pas encore.** 1er essai : posture apprise trop haute (le journal montre que le robot a
  rejoint exactement la posture apprise) → prise réapprise (21,7°, levée verticale). 2e essai :
  cycle jusqu'au bout mais **la pince n'a pas serré l'éponge du tout**. Cause non identifiée
  (pince bloquée ? largeur/force OnRobot ? éponge pas entre les doigts ?).
- À savoir : `teach_grasp.py` n'enregistre que le **dernier `g`** ; et le programme refuse de partir
  si le bras est laissé trop près de la table (aucune commande envoyée) → le remonter en freedrive.

**Fin de séance**
- **Canette traitée comme la bouteille** (décision de l'équipe) : `taught_route.route_aliases`
  = `{"aluminum_can": "plastic_bottle"}` → même case, même prise, même dépôt (bac gauche). Aucune
  posture propre à la canette n'est utilisée, donc l'arrêt de protection de son apprentissage ne
  la bloque plus (commit `e2ece3c`).
- **Position d'observation apprise** : le cycle finit par `RETURN_TO_OBSERVE`, bras replié près du
  pied (commit `f28b6a5`).
- **Table non enregistrée** (choix de l'équipe) : la boîte enregistrée avec `teach_cell.py` (points
  lus = bride, pas le plateau) bloquait toutes les prises → retirée ; on garde le plancher virtuel
  à 0,35 m. Le planificateur ne connaît donc pas la table : la protection repose sur les postures
  apprises, le pas-à-pas et l'arrêt d'urgence.
- **Bug corrigé dans `teach_cell.py` et `teach_grasp.py`** : la position du bras était lue à
  l'affichage de l'invite et non à l'appui sur la touche ; un bras déplacé pendant l'attente était
  ignoré (commit `e4b9e36`).
- **Un collègue a testé tous les cas sur le vrai robot : tous les tests ont marché** (objets un par
  un, saisie enchaînée `--confirm-before-picking`, Ctrl+C, deux objets dans la zone).
- La vision en direct n'a pas été lancée : le mode `--zed` a besoin du ZED SDK, donc d'une carte
  NVIDIA (le laptop a une carte AMD).

**Divers**
- Messages clairs au lieu d'une trace Python quand le robot est injoignable.
- Docs de connexion (README, checklist).

---

## Ce qui reste à faire

### À faire — implémenter la partie automatique du pipeline

Objectif : une commande, et le robot enchaîne seul **caméra → reconnaissance (YOLO) → choix du bac
→ prise → dépôt → retour en observation**, objet après objet, sans Entrée à chaque étape.

Aujourd'hui, la chaîne existe dans `run_sorting.py --zed`, mais elle n'a jamais tourné en direct et
elle demande encore l'opérateur à chaque cycle (`yes` × 2, Entrée par étape). Il reste à :

- [ ] **Faire tourner la caméra en direct** avec le robot : soit sur le PC de la ZED (carte NVIDIA,
      `--zed`), soit sur le laptop en lisant la ZED comme une webcam USB, sans le SDK (mode à écrire).
- [ ] **Enlever les confirmations manuelles** une fois la boucle validée en direct : les questions
      `yes` par objet et le pas-à-pas (`--no-step`). Ce dernier exige `validated_on_hardware` et
      `collision_geometry_measured` à `true` dans `configs/real_cell/motion_config.json` ; la table
      n'étant pas modélisée, c'est une décision de l'équipe (vérifier les plans de sécurité du
      pendant avant).
- [ ] **Vérifier la saisie avec les capteurs de la pince** au lieu d'un délai fixe : attendre
      « prête » (entrée outil 1) et refuser de lever si aucun objet n'est détecté (entrée outil 0).
      Indispensable en automatique : personne ne regarde plus chaque saisie.
- [ ] **Démo** : `--cycles 4`, objets posés un par un sur leur case pendant que le bras est en
      observation.

### Plus tard (option B) — le robot trouve où est l'objet

- [ ] Outil d'étalonnage caméra → robot (marqueur sur la pince, 10–15 poses,
      `cv2.calibrateRobotWorldHandEye`), validé à < ~1 cm, puis décalage limité de la prise
      (`max_pick_offset_m`, `--placement measured --camera-to-base FICHIER.json`).
- [ ] Photographier les réglages du pendant (TCP, Safety, OnRobot) pour le rapport.

### Hors périmètre pour l'instant

- Brosse à dents : aucune posture apprise (ni prise ni dépôt) → refusée.

---

## Questions encore ouvertes

| Question | Statut |
|---|---|
| TCP du pendant (0 ou 150 mm) ? | **Réglé** : 0 |
| Sortie de commande de la pince ? | **Réglé** : sortie outil 0, ON = fermer |
| Montage de la pince, longueur réelle ? | non mesuré (table non modélisée) |
| Cause de l'arrêt de protection de la canette ? | contourné : canette = trajet de la bouteille |
| Cause du blocage de la pince (sécurité RG6 ?) | probable, non prouvé |
| Données d'étalonnage caméra ? | aucune, à produire (option B) |
| Date de la démo ? | à préciser |

## Où trouver quoi

- Procédure pas à pas au labo : [`ROBOT_CHECKLIST_FR.md`](ROBOT_CHECKLIST_FR.md)
- Commandes et fonctionnement : [`../README.md`](../README.md)
- Historique des changements : [`CHANGELOG.md`](CHANGELOG.md)
- Journal des cycles réels : `logs/robot_sorting.jsonl` (non versionné) ; vérification du
  2 novembre : `logs/check_robot_2026-11-02.txt`
