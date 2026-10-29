import json
from wingman_exploration.mission_parser import parse_mission, normalize_command


def test_text_altitude():
    m, _ = parse_mission("explore the maze and return to home while maintaining height of 0.75m")
    assert (m["type"], m["then"], m["altitude"]) == ("explore", "return", 0.75)
    m, _ = parse_mission("fly at altitude 75 cm and explore")
    assert m["altitude"] == 0.75 and m["type"] == "explore"
    m, _ = parse_mission("explore, keeping a height of 5 meters")
    assert m["altitude"] == 2.0                     # clamped
    m, _ = parse_mission("find the person at 1 m and come back")
    assert (m["type"], m["target"], m["altitude"]) == ("find", "person", 1.0)


def test_no_false_altitude():
    m, _ = parse_mission("find a door 2 m away")
    assert m["altitude"] is None and "door" in m["target"]
    assert parse_mission("go to 4 -10")[0]["xy"] == [4.0, -10.0]


def test_shortest_path_is_exploration_and_lands_at_start():
    m, _ = parse_mission("explore the maze and find the shortest path and land at the starting point")
    assert (m["type"], m["then"]) == ("explore", "return")


def test_json():
    m, _ = parse_mission('{"type":"explore","then":"return","altitude":0.75}')
    assert m["altitude"] == 0.75
    assert parse_mission('{"type":"explore","altitude":"abc"}')[0] is None
    assert parse_mission('{"type":"explore","then":"nope"}')[0] is None
    assert parse_mission('{"type":"explore","altitude":0.1}')[0]["altitude"] == 0.4


def test_gateway_normalize():
    canon, m, err = normalize_command('{"id":"1","type":"explore","then":"return","altitude":0.75,"evil":"x"}')
    assert err == "" and "evil" not in json.loads(canon) and json.loads(canon)["altitude"] == 0.75
    canon, m, err = normalize_command('{"id":"2","prompt":"explore the maze and return home at 0.75 m"}')
    assert json.loads(canon)["altitude"] == 0.75 and json.loads(canon)["type"] == "explore"
    canon, m, err = normalize_command('{"id":"3","prompt":"explore the maze","altitude":0.9}')
    assert json.loads(canon)["altitude"] == 0.9
    assert normalize_command("{bad")[0] is None
    assert normalize_command("x" * 5000)[0] is None
    assert normalize_command('{"type":"launch missiles"}')[0] is None
    assert normalize_command('[1,2]')[0] is None
