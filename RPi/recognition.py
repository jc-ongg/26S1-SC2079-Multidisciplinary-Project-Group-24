"""Normalize image-API responses into numeric MDP IDs for TARGET frames."""
IMAGE_LABELS = {
    **{n + 10: str(n) for n in range(1, 10)},
    **dict(zip(range(20, 36), 'ABCDEFGHSTUVWXYZ')),
    36: 'UP', 37: 'DOWN', 38: 'RIGHT', 39: 'LEFT', 40: 'CIRCLE',
}
LABEL_TO_ID = {label: number for number, label in IMAGE_LABELS.items()}


def normalize_recognition(payload, obstacle_id=None):
    if not isinstance(payload, dict):
        raise ValueError('Image API returned no result object')
    context = payload.get('target_obstacle_id')
    if context not in (None, '') and obstacle_id is not None and str(context) != str(obstacle_id):
        raise ValueError('Image API returned a result for a different obstacle')
    if payload.get('accepted') is False or str(payload.get('image_id', '')).upper() in ('NA', 'BULLSEYE'):
        return {**payload, 'image_id': 'NA', 'obstacle_id': 'NA', 'mdp_image_id': None}
    if not payload.get('image_id') and payload.get('results'):
        payload = max(payload['results'], key=lambda item: float(item.get('conf', 0)))
    label = str(payload.get('image_id', '')).strip().upper()
    number = payload.get('mdp_image_id')
    if number is None:
        # Prefer the label over React's class-index obstacle_id field.
        number = LABEL_TO_ID.get(label, payload.get('obstacle_id'))
    if number is None and label.isdigit():
        number = label
    try:
        number = int(number)
    except (TypeError, ValueError):
        raise ValueError('Image API returned no valid numeric image ID') from None
    if number not in IMAGE_LABELS:
        raise ValueError(f'Image API returned unsupported image ID {number}')
    return {**payload, 'image_id': IMAGE_LABELS[number], 'obstacle_id': str(number),
            'mdp_image_id': number}
