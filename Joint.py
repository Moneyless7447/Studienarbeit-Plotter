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

if __name__ == '__main__':
    eins_eins_eins=Joint(1.0, 1.0, 1.0, 1.0, "1.1.1", "", [])
    eins_eins_zwei=Joint(1.0, 1.0, 1.0, 1.0, "1.1.1", "", [])
    eins_eins=Joint(1.0, 1.0, 1.0, 1.0, "1.1", "", [eins_eins_eins])