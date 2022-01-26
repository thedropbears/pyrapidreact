# pyrapidreact

The Drop Bears' robot code for _FIRST_ Rapid React (FRC 2022).

## Install dependencies

### On a computer

With [`pipenv`](https://pipenv.pypa.io/en/latest/):

    pipenv install

With plain `pip`:

    pip3 install -r requirements-sim.txt

### For the roboRIO

```sh
# Online:
robotpy-installer download-python
robotpy-installer download -r requirements.txt

# On the robot network:
robotpy-installer install-python
robotpy-installer install -r requirements.txt
```

## pre-commit

This project has [pre-commit.com](https://pre-commit.com) set up.

## Code style
This codebase adheres to the code style enforced by the black autoformatter:

    black .

This is enforced by CI. To install this:

    pip3 install black

See [PEP 8](https://www.python.org/dev/peps/pep-0008/) on naming conventions.

[Docstrings should follow Google style](https://google.github.io/styleguide/pyguide.html#383-functions-and-methods).
See also [PEP 257](https://www.python.org/dev/peps/pep-0257/).

## Run

### Simulation (desktop)

    ./robot.py sim

### Deploy to robot

    ./robot.py deploy

This project is configured to automatically deploy to 4774's robot.
