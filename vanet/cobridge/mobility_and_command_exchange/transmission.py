"""Build radio transmissions without granting receivers omniscient state."""
import json


def wireless_transmission(sender, recipient, payload):
    if not isinstance(sender, str) or not sender or not isinstance(recipient, str) or not recipient:
        raise ValueError('Wireless endpoints must have stable string identities')
    if not isinstance(payload, dict):
        raise ValueError('Application payload must be an object')
    # Also rejects NaN and nonserializable runtime objects before submitting a step.
    json.dumps(payload, allow_nan=False)
    return dict(sender=sender, recipient=recipient, payload=dict(payload))
