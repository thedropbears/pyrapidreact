from components.indexer import indexer
from components.intake import Intake
from magicbot import StateMachine, default_state, state, timed_state, tunable


class IndexerController(StateMachine):
    intake: Intake,
    indexer: Indexer,

    wants_to_intake = tunable(False)
    wants_to_fire = tunable(False)
    trapped = tunable(False)

    @default_state
    def stopped(self) -> None:
        self.indexer.set_piston(False)
        self.indexer.set(0, 0, 0)
        self.intake.set(0)
        if self.check_firing():
            self.next_state("firing")
        elif self.wants_to_intake or (self.indexer.has_back() and not self.indexer.has_front):
            self.next_state("intaking")
        
    @state
    def intaking(self) -> None:
        self.indexer.set(1, 0, 0)
        self.intake.set(1)
        if self.indexer.has_front():
            self.next_state("reading")

    @state
    def reading(self, state_tm) -> None:
        self.indexer.set(0, 0, 0)
        self.intake.set(0)
        if state_tm > 0.1:
            if self.indexer.is_front_ours():
                self.next_state("indexing")
            elif not self.trapped:
                self.trapped = True
                self.next_state("trapping")
            else:
                self.next_state("clearing")

    @timed_state(duration=0.5, next_state="stopped")
    def clearing(self) -> None:
        self.indexer.set(-1, 0, 0)
        self.intake.set(-1)

    @timed_state(duration=2.0, next_state="stopped")
    def indexing(self) -> None:
        self.indexer.set(1, 1, 0)
        if self.indexer.has_back():
            self.wants_to_intake = False
            self.next_state("stopped")

    @timed_state(duration=0.5, next_state="stopped")
    def firing(self) -> None:
        self.indexer.set(0, 1, 1)

    @timed_state(duration=0.5, next_state="stopped")
    def trapping(self, state_tm) -> None:
        self.indexer.set_piston(True)
        if (state_tm > 0.1):
            self.indexer.set(1, 1, 0)

    def check_firing(self) -> bool:
        return self.wants_to_fire and self.indexer.has_back()

