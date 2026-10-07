"""Coalesce straight commands without crossing a turn or image capture."""

def coalesce(entries):
    output = []
    previous = None
    for command, endpoint in entries:
        if command.startswith('center,') and int(command.split(',')[3]) == 0:
            continue
        if command.startswith(('SNAP', 'FIN')):
            if previous is not None:
                output.append(previous)
                previous = None
            output.append((command, endpoint))
            continue
        if previous is None:
            previous = (command, endpoint)
            continue
        current = command.split(',')
        earlier = previous[0].split(',')
        if current[0] == earlier[0] == 'center':
            signed = lambda fields: int(fields[3]) * (1 if fields[2] == 'forward' else -1)
            distance = signed(current) + signed(earlier)
            previous = None if distance == 0 else (
                f"center,0,{'forward' if distance > 0 else 'reverse'},{abs(distance)}", endpoint)
        else:
            output.append(previous)
            previous = (command, endpoint)
    if previous is not None:
        output.append(previous)
    return output
