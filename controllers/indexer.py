from components.indexer import Indexer
from components.intake import Intake
from magicbot import (
    StateMachine,
    default_state,
    state,
    timed_state,
    tunable,
    will_reset_to,
)


class IndexerController(StateMachine):
    intake: Intake
    indexer: Indexer

    wants_to_intake = tunable(False)
    wants_to_fire = will_reset_to(False)
    trapped = tunable(False)
    ignore_colour = tunable(False)
    has_back = tunable(False)

    @default_state
    def stopped(self) -> None:
        self.indexer.set_chute(False)
        self.indexer.set(0, 0)
        self.intake.set(0)
        if self.check_firing():
            self.next_state("firing")
        elif self.wants_to_intake and not (
            (self.indexer.has_back() or self.has_back) and self.indexer.has_front()
        ):
            self.next_state("intaking")
        elif self.indexer.has_front() and not self.indexer.has_back():
            self.next_state("indexing")

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        self.indexer.set(1, 0)
        self.intake.set(1)
        if self.check_firing():
            self.next_state("firing")
        elif self.indexer.has_front():
            self.next_state("reading")
        elif not self.wants_to_intake:
            self.next_state("stopped")

    @state(must_finish=True)
    def reading(self, state_tm) -> None:
        self.indexer.set(0, 0)
        self.intake.set(0)
        if state_tm > 0.1:
            if self.indexer.is_front_ours() or self.ignore_colour:
                print("moving to back")
                self.next_state("indexing")
            elif not self.trapped:
                self.trapped = True
                print("clearing")
                # self.next_state("trapping")
                self.next_state("clearing")
            else:
                print("clearing")
                self.next_state("clearing")

    @timed_state(duration=1, next_state="stopped", must_finish=True)
    def clearing(self) -> None:
        self.indexer.set(-1, 0)
        self.intake.set(-1)
        self.wants_to_intake

    @timed_state(duration=10.0, next_state="stopped", must_finish=True)
    def indexing(self) -> None:
        if self.indexer.has_back() or self.has_back:
            self.wants_to_intake = False
            self.has_back = True
            self.next_state("stopped")
            print("finished indexing")
            return
        self.intake.set(0)
        self.indexer.set(1, 1)

    @timed_state(duration=0.5, next_state="stopped", must_finish=True)
    def firing(self) -> None:
        self.indexer.set(0, 1)
        self.has_back = False

    @timed_state(duration=0.5, next_state="stopped", must_finish=True)
    def trapping(self, state_tm) -> None:
        self.indexer.set_chute(True)
        if state_tm > 0.1:
            self.indexer.set(1, 1)

    def check_firing(self) -> bool:
        return self.wants_to_fire and (self.indexer.has_back() or self.has_back)
