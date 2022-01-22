from magicbot.state_machine import AutonomousStateMachine, state


class AutoBase(AutonomousStateMachine):
    def __init__(self):
        super().__init__()

    @state(first=True)
    def starting(self):
        pass
