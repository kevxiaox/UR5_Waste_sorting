# Checklist sur place (UR5)

On ne passe à une étape que si la précédente est OK. Les étapes 1 à 3 **ne font pas bouger le robot**.
Dès que le robot bouge : une personne la main sur l'arrêt d'urgence, curseur de vitesse à 10-20 %.

## 1. Connexion

- [ ] Pendant, *Setup Robot → Network* : IP `147.250.35.40`, masque `255.255.255.0`, passerelle `147.250.35.245`, statique. **Ne pas passer en DHCP.**
- [ ] PC branché **en filaire** (adaptateur USB → RJ45) ; le Wi-Fi de l'école ne voit pas le robot. VPN déconnecté.
  - soit sur le même réseau que le robot (prise murale / switch), adaptateur en automatique : `ipconfig` doit donner une adresse `147.250.35.x` ;
  - soit câble direct vers le boîtier de contrôle, adresse fixe du PC `147.250.35.200`, masque `255.255.255.0`, sans passerelle
    (*Paramètres → Réseau et Internet → Ethernet → Attribution d'adresse IP → Manuel*). Uniquement en câble direct ; rebrancher ensuite le câble du robot.
- [ ] `ping 147.250.35.40` répond (si Windows demande d'autoriser Python sur le réseau : accepter).
- [ ] `pip install -r requirements.txt` puis `python tools/check_robot.py --ip 147.250.35.40` :
  `robot mode 7 (RUNNING)`, erreur de position < 3 mm (attendu ≈ 2 mm), `collision model ...: valid`.

## 2. Réglages à relever sur le pendant (photos)

- [ ] *Installation → TCP* : TCP actif. Il doit être égal à `tcp_pose` de `configs/real_cell/motion_config.json` (0 par défaut, valeur lors de l'apprentissage des poses). Sinon : corriger la config (les poses apprises restent valables).
- [ ] *Installation → OnRobot* (si présent) : mode de commande de la RG6, compensation de profondeur.
- [ ] *Installation → Safety* : limites articulaires, vitesse outil, **plans de sécurité**.

## 3. Cellule (fait le 2 nov.)

- [x] Prises réapprises pour chaque objet (`tools/teach_grasp.py`), cases marquées au scotch.
- [x] **Position d'observation** (`python tools/teach_cell.py --ip 147.250.35.40`, `o` puis `s`) : bras replié, hors du champ de la caméra.
- [x] Table **non modélisée** (décision de l'équipe) : plancher virtuel à 0,35 m conservé. Ne pas utiliser `b` sur le plateau : avec TCP = 0 le point lu est la **bride**, pas le bout des doigts.

## 3 bis. Robot réel sans caméra (détections enregistrées) — validé le 2 nov.

- [x] Un objet à la fois sur sa case, bras au-dessus de la table, puis :
  `python run_sorting.py --replay fixtures/vision/<objet>_stable.jsonl --robot-ip 147.250.35.40 --allow-unmeasured --confirm-before-picking`
  avec `<objet>` = `plastic_bottle`, `aluminum_can` (sur la case de la bouteille), `paper_cup`, `sponge`.
- [x] `fixtures/vision/two_objects_ambiguous.jsonl` → refus, aucun mouvement ; Ctrl+C pendant un mouvement → arrêt contrôlé.
- Détails et dépannage : README, section *Testing on the real robot*.

## 4. Vision seule (robot simulé)

- [ ] `pip install -r requirements-vision.txt`, ZED SDK + API Python installés.
- [ ] `python run_sorting.py --zed` : les zones collent à la table → **V**. Éponge au point de prise → `ready sponge` puis cycle simulé `COMPLETED`.
- [ ] Deux objets → `ambiguous`, rien ne se passe ; objet peu sûr (< 0,60) → refusé.

## 5. Robot réel avec la caméra, pas-à-pas (à faire)

- [ ] Sur le PC de la ZED (carte NVIDIA) : `python run_sorting.py --zed --robot-ip 147.250.35.40 --confirm-before-picking --allow-unmeasured`
- [ ] Ordre : éponge, gobelet, bouteille, canette (posée sur la case de la bouteille : elle suit exactement le trajet de la bouteille, `taught_route.route_aliases`).
- [ ] À chaque étape : approche = posture apprise, descente quasi droite, pince qui s'ouvre puis se ferme (sinon niveaux inversés dans `configs/gripper_io.json`), objet soulevé, dépôt dans le bon bac.
- [ ] Tester **Ctrl+C** pendant un mouvement (arrêt contrôlé) et deux objets dans la zone (refus).

## 6. Après validation

- [ ] `validated_on_hardware` et `collision_geometry_measured` à `true` (`configs/real_cell/motion_config.json`) — décision de l'équipe, la table n'étant pas modélisée (vérifier les plans de sécurité du pendant). `observe.measured` est déjà à `true`.
- [ ] Monter le curseur progressivement ; pour la démo : `--cycles 4`, éventuellement `--no-step`.
- [ ] Garder `logs/robot_sorting.jsonl` et les sorties de `check_robot.py`.
