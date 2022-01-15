from json import load

# with open("./example_robot_dh.json") as data_json:
#    data = load(data_json)

data = {'robot':[{
    "title": "Beta1-Gelenk",
    "type": "rotation",
    "angle": "0",
    "length": "2.3",
    "offset": "1",
    "twist": "3.14159",
    "children": [
        {
            "title": "Gamma1-Gelenk",
            "type": "rotation",
            "angle": "0",
            "length": "1",
            "offset": "3",
            "twist": "0.7854"
        },
        {
            "title": "Gamma1-Gelenk",
            "type": "rotation",
            "angle": "-1.57078",
            "length": "0.5",
            "offset": "2",
            "twist": "0"
        }
    ]
}]}

from dataclasses import dataclass


@dataclass
class Joint:
    name: str
    angle: float
    length: float
    offset: float
    twist: float
    title: str
    type: str
    children: list
    previous: str


def build_tree(data_list: list):
    def _build_tree(data: dict, result: list = list(), key: str = "1", previous_key: str = "0"):
        if 'children' not in data:
            child = Joint(key, data["angle"], data["length"], data["offset"], data["twist"], data["title"],
                          data["type"],
                          list(), previous_key)
            result.append(child)
            return child
        else:
            parent = Joint(key, data["angle"], data["length"], data["offset"], data["twist"], data["title"],
                           data["type"],
                           list(), previous_key)

            for index, value in enumerate(data['children']):
                new_key = f"{key}.{index + 1}"
                child = _build_tree(data=value, result=result, key=new_key, previous_key=key)
                parent.children.append(child)
            result.append(parent)
            return

    result = list()
    for index, value in enumerate(data_list):
        _build_tree(data=value, result=result, key=f"{index + 1}")
    return result


"""
def build_tree(data_list: list):
    def _build_tree(data: dict, result: dict = dict(), key: str = "1", previous_key: str = "0"):
        if 'children' not in data:
            result[key] = data
            result[key]["previous"] = previous_key
            return
        else:
            result[key] = {"angle": data["angle"], "length": data["length"], "offset": data["offset"],
                           "twist": data["twist"], "name": data["name"], "type": data["type"], "children": list(),
                           "previous": previous_key}

            Joint(data["angle"], data["length"], data["offset"], data["twist"], data["title"], data["type"], list(),
                  previous_key)

            for index, value in enumerate(data['children']):
                new_key = f"{key}.{index + 1}"
                result[key]["children"].append(new_key)
                _build_tree(data=value, result=result, key=new_key, previous_key=key)
            return
    result = dict()
    for index, value in enumerate(data_list):
        _build_tree(data=value, result=result, key=f"{index + 1}")
    return result
"""

if __name__ == '__main__':
    print(build_tree(data["robot"]))
