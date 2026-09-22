"""Render the actual HA package templates with mocked HA state access.

This tests fusion behavior, not Home Assistant's full integration schema.
Run with an environment providing PyYAML and Jinja2.
"""
import math
from pathlib import Path
import yaml
from jinja2 import Environment, StrictUndefined

ROOT = Path(__file__).resolve().parents[1]
package = yaml.safe_load((ROOT / 'home_assistant_combined_people_counter.yaml').read_text())
dashboard = yaml.safe_load((ROOT / 'home_assistant_combined_people_counter_dashboard.yaml').read_text())
entities = package['template'][0]['sensor']
binary = package['template'][1]['binary_sensor']
side = 'number.office_dorteller_nede_dorteller_nede_people_inside'
top = 'number.office_dorteller_oppe_dorteller_oppe_people_inside'
assert entities[0]['attributes']['source_entity'] == side
assert entities[1]['attributes']['source_entity'] == top
physical = {f'{domain}.office_dorteller_{loc}_dorteller_{loc}_{suffix}'
            for loc in ['nede', 'oppe']
            for domain, suffix in [('number', 'people_inside'), ('sensor', 'confirmed_in'),
                ('sensor', 'confirmed_out'), ('binary_sensor', 'counter_ready'),
                ('binary_sensor', 'sensor_covered'), ('sensor', 'system_status'),
                ('sensor', 'unsure_detection_in'), ('sensor', 'unsure_detection_out')]}
state = {}
def is_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
env = Environment(undefined=StrictUndefined)
env.globals.update(states=lambda e: state.get(e, 'unknown'), is_number=is_number)
def render(template):
    return env.from_string(template).render().strip()
def evaluate(entity, domain):
    available = render(entity.get('availability', '{{ true }}')).lower() == 'true'
    result = render(entity['state']) if available else 'unavailable'
    state[domain + '.' + entity['unique_id']] = result
    return result
cases = [
    ('0','0','0','agreement'), ('1','0','1','conflict'),
    ('0','1','1','conflict'), ('2','3','3','conflict'),
    ('8','8','8','agreement'), ('unavailable','4','4','oppe_only'),
    ('5','unknown','5','nede_only'), ('unavailable','unknown','unavailable','offline'),
    ('-1','3','3','oppe_only'), ('1.5','2','2','oppe_only'),
    ('nan','inf','unavailable','offline'), ('garbage','','unavailable','offline'),
    ('0','unknown','0','nede_only'),
]
for a,b,expected,status in cases:
    state.clear()
    state.update({side:a,top:b,'input_number.roode_test_actual_people':'2'})
    for entity in entities: evaluate(entity, 'sensor')
    for entity in binary: evaluate(entity, 'binary_sensor')
    assert state['sensor.roode_combined_people_inside'] == expected, (a,b,state)
    assert state['sensor.roode_counter_fusion_status'] == status, (a,b,state)
    assert state['binary_sensor.roode_room_occupied'] == (
        'unavailable' if expected == 'unavailable' else str(int(expected)>0)
    )
    if status == 'offline':
        assert state['sensor.roode_test_error'] == 'unavailable'
    script = package['script']['roode_set_both_counts']
    for k,v in script['sequence'][0]['variables'].items(): env.globals[k] = render(v)
    assert render(script['sequence'][1]['value_template']).lower() == str(status in ['agreement','conflict']).lower()
# Check the dashboard only refers to entities supplied by this package.
known = {'sensor.'+e['unique_id'] for e in entities}
known |= {'binary_sensor.'+e['unique_id'] for e in binary}
known.add('input_number.roode_test_actual_people')
known |= physical
seen = set()
def walk(obj):
    if isinstance(obj,dict):
        if 'entity' in obj:
            assert obj['entity'] in known, obj['entity']
            seen.add(obj['entity'])
        for k,v in obj.items():
            if k=='entities':
                for item in v:
                    if isinstance(item,str):assert item in known,item
            walk(v)
    elif isinstance(obj,list):
        for item in obj:walk(item)
walk(dashboard)
assert package['script']['roode_set_both_counts']['sequence'][2]['target']['entity_id'] == [side,top]
assert physical <= seen, physical - seen
assert 'integration_entities(' not in str(package)
state.clear()
for entity in entities[:2]:
    assert render(entity['availability']).lower() == 'false'
print(f'PASS: {len(cases)} fusion cases, availability, occupancy, correction guard and dashboard entity references')

# Short filenames are the installable copies of the same package/dashboard.
assert yaml.safe_load((ROOT / 'dorteller-ha-pakke.yaml').read_text()) == package
assert yaml.safe_load((ROOT / 'dorteller-ha-dashboard.yaml').read_text()) == dashboard
