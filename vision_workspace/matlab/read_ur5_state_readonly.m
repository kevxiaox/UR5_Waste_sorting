function state = read_ur5_state_readonly(host, port)
%READ_UR5_STATE_READONLY Subscribe to controller outputs; no control inputs.
%   state = read_ur5_state_readonly("147.250.35.40")
%   read_ur5_state_readonly("selftest") runs offline codec checks only.
%
% Only RTDE protocol negotiation, controller-version query, OUTPUT recipe
% setup, and DATA synchronization start are sent. No input recipes, register
% values, URScript, dashboard commands, I/O writes or robot program start.
% RTDE synchronization start means data streaming, not robot motion.
% TCP values reflect the currently configured tool, whose geometry is NOT
% independently validated. They are NOT camera/base calibration pairs.
%
% Protocol source:
% https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html

if nargin < 1
    host = "147.250.35.40";
end
if nargin < 2
    port = 30004;
end
host = string(host);
if host == "selftest"
    codecSelftest();
    state = struct('offline_tests_passed', true);
    return
end
validateattributes(port, {'numeric'}, {'scalar','integer','>=',1,'<=',65535});
fields = 'timestamp,actual_q,actual_TCP_pose,tcp_offset';
expectedTypes = 'DOUBLE,VECTOR6D,VECTOR6D,VECTOR6D';
fprintf('Subscribing to RTDE OUTPUTS at %s:%d. Robot execution DISABLED.\n', host, port);
client = tcpclient(host, port, 'Timeout', 5, 'ConnectTimeout', 5);
try
    % Protocol version 2 (big-endian uint16).
    sendPacket(client, 86, uint8([0 2]));
    accepted = waitPacket(client, 86);
    if ~isequal(accepted, uint8(1))
        error('URReadonly:Protocol', 'Controller rejected RTDE v2; no fallback or setting changes made.');
    end
    sendPacket(client, 118, uint8([]));
    versionPayload = waitPacket(client, 118);
    if numel(versionPayload) ~= 16
        error('URReadonly:Version', 'Unexpected controller version packet.');
    end
    versionNumbers = double(reshape(versionPayload, 4, 4));
    versionNumbers = [16777216 65536 256 1] * versionNumbers;

    % Output recipe only. Deliberately NEVER send type 73 (SETUP_INPUTS).
    sendPacket(client, 79, [encodeDoubles(10), uint8(fields)]);
    recipe = waitPacket(client, 79);
    if numel(recipe) < 2 || recipe(1) == 0 || ~strcmp(char(recipe(2:end)), expectedTypes)
        error('URReadonly:Recipe', 'Controller output fields rejected: %s', char(recipe));
    end
    recipeID = recipe(1);
    sendPacket(client, 83, uint8([]));
    accepted = waitPacket(client, 83);
    if ~isequal(accepted, uint8(1))
        error('URReadonly:Stream', 'Controller refused output streaming.');
    end
    data = waitPacket(client, 85);
    values = decodeState(data, recipeID);
    clear client
catch problem
    clear client
    rethrow(problem)
end

state = struct;
state.schema_version = 1;
state.received_at_utc = char(datetime('now', 'TimeZone','UTC', ...
    'Format',"yyyy-MM-dd'T'HH:mm:ss.SSSXXX"));
state.robot_ip = char(host);
state.controller_version = sprintf('%d.%d.%d.%d', versionNumbers);
state.controller_timestamp_s = values(1);
state.actual_q_rad = values(2:7);
state.actual_q_deg = rad2deg(values(2:7));
state.actual_tcp_pose_base_m_rotvec_rad = values(8:13);
state.active_tcp_offset_flange_m_rotvec_rad = values(14:19);
state.robot_execution_enabled = false;
state.tool_geometry_validated = false;
state.camera_to_base_calibrated = false;
state.note = 'Controller-reported state of configured TCP only; not a calibration pair or grasp target.';
fprintf('Controller version: %s\n', state.controller_version);
fprintf('Actual joints [Base Shoulder Elbow Wrist1 Wrist2 Wrist3] (deg):\n');
disp(state.actual_q_deg);
fprintf('Actual TCP in robot Base [x y z rx ry rz] (meters / rotation-vector radians):\n');
disp(state.actual_tcp_pose_base_m_rotvec_rad);
fprintf('Active TCP offset from flange [x y z rx ry rz] (meters / rotation-vector radians):\n');
disp(state.active_tcp_offset_flange_m_rotvec_rad);
folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'robot_state_readonly');
if ~isfolder(folder)
    mkdir(folder);
end
path = [tempname(folder), '.json'];
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('URReadonly:Save', 'Could not create state record in %s.', folder);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(state, 'PrettyPrint',true), 'char');
clear cleanup
fprintf('Saved: %s\n', path);
end

function sendPacket(client, kind, payload)
% Restrict even the transport helper to non-control message types.
write(client, encodePacket(kind, payload), 'uint8');
end

function bytes = encodePacket(kind, payload)
if ~ismember(kind, [86 118 79 83])
    error('URReadonly:ForbiddenMessage', 'Control/input message type %d is forbidden.', kind);
end
payload = reshape(uint8(payload), 1, []);
count = numel(payload) + 3;
if count > 65535
    error('URReadonly:PacketSize', 'Packet too large.');
end
bytes = [uint8([floor(count/256), mod(count,256), kind]), payload];
end

function payload = waitPacket(client, expected)
started = tic;
for attempt = 1:32
    if toc(started) > 5
        break
    end
    header = readExactly(client, 3);
    count = double(header(1))*256 + double(header(2));
    if count < 3 || count > 8192
        error('URReadonly:Header', 'Invalid RTDE frame length %d.', count);
    end
    payload = readExactly(client, count-3);
    if header(3) == expected
        return
    end
    if header(3) ~= 77 && header(3) ~= 85
        error('URReadonly:UnexpectedReply', 'Unexpected RTDE reply %d.', header(3));
    end
end
error('URReadonly:ReplyTimeout', 'No expected RTDE reply %d received.', expected);
end

function bytes = readExactly(client, count)
bytes = zeros(1, count, 'uint8');
used = 0;
started = tic;
while used < count
    available = min(client.NumBytesAvailable, count-used);
    if available > 0
        chunk = read(client, available, 'uint8');
        bytes(used+1:used+available) = reshape(chunk,1,[]);
        used = used + available;
    elseif toc(started) >= 5
        error('URReadonly:ReadTimeout', 'Incomplete RTDE packet; no state accepted.');
    else
        pause(0.005);
    end
end
end

function bytes = encodeDoubles(values)
values = double(values);
[~,~,endianness] = computer;
if endianness == 'L'
    values = swapbytes(values);
end
bytes = reshape(typecast(values(:), 'uint8'),1,[]);
end

function values = decodeState(payload, recipeID)
if numel(payload) ~= 153 || payload(1) ~= recipeID
    error('URReadonly:Data', 'Unexpected output recipe ID or data length.');
end
values = reshape(typecast(uint8(payload(2:end)), 'double'),1,[]);
[~,~,endianness] = computer;
if endianness == 'L'
    values = swapbytes(values);
end
if any(~isfinite(values))
    error('URReadonly:Nonfinite', 'Non-finite robot state; no state accepted.');
end
end

function codecSelftest()
assert(isequal(encodePacket(86, uint8([0 2])), uint8([0 5 86 0 2])));
assert(isequal(encodeDoubles(10), uint8([64 36 0 0 0 0 0 0])));
expected = [123.5, -1, 0.2, 0.3, 0.4, 0.5, 0.6, ...
    0.1, -0.2, 0.7, 3.1, 0, -0.1, 0, 0, 0.15, 0, 0, 0];
payload = [uint8(7), encodeDoubles(expected)];
assert(isequal(decodeState(payload,uint8(7)), expected));
assertThrows(@() decodeState(payload,uint8(8)), 'URReadonly:Data');
assertThrows(@() decodeState(payload(1:end-1),uint8(7)), 'URReadonly:Data');
assertThrows(@() decodeState([uint8(7),encodeDoubles([NaN,expected(2:end)])],uint8(7)), ...
    'URReadonly:Nonfinite');
for forbidden = [73 85 80]
    assertThrows(@() encodePacket(forbidden,uint8([])), 'URReadonly:ForbiddenMessage');
end
fprintf('Offline codec checks PASSED; no network connection created.\n');
end

function assertThrows(action, expectedID)
try
    action();
catch problem
    assert(strcmp(problem.identifier, expectedID), 'Unexpected error identifier.');
    return
end
error('URReadonly:MissingError', 'Expected rejection did not occur.');
end
