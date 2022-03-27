from components.indexer import Indexer
from magicbot import (
    StateMachine,
    default_state,
    state,
    timed_state,
    tunable,
)
import wpiutil.log

from controllers.shooter import ShooterController


class IndexerController(StateMachine):
    indexer: Indexer
    shooter_control: ShooterController
    data_log: wpiutil.log.DataLog

    wants_to_intake = tunable(False)
    ignore_colour = tunable(False)
    catflap_active = tunable(False)

    def setup(self) -> None:
        self.log_colour = wpiutil.log.StringLogEntry(self.data_log, "/indexer/colour")

    def stop(self) -> None:
        self.next_state("stopping")

    @default_state
    def stopped(self) -> None:
        # By default the indexer does nothing and has the cat flap closed, so we can do nothing too!

        # We need to check if we should be moving a ball from the tunnel to the chimney
        if (
            self.indexer.has_cargo_in_tunnel()
            and not self.indexer.has_cargo_in_chimney()
            and not self.shooter_control.current_state == "firing"
        ):
            # We can just check the indexer prox sensors, because opposition cargo is rejected before
            # returning to the "stopped" state, so it is guaranteed to be our cargo
            self.next_state("transferring_to_chimney")
        elif self.wants_to_intake and self.indexer.ready_to_intake():
            self.next_state("intaking")

    @state(must_finish=True)
    def stopping(self) -> None:
        self.indexer.reset_cargo_colour()
        self.wants_to_intake = False
        self.next_state("stopped")

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        self.indexer.read_cargo_colour()
        if self.indexer.has_cargo_in_tunnel():
            self.next_state("reading")
            return
        self.indexer.run_tunnel_motor(Indexer.Direction.FORWARDS)

    @state(must_finish=True)
    def reading(self, state_tm: float) -> None:
        self.indexer.read_cargo_colour()
        colour = self.indexer.get_cargo_colour()
        if state_tm < 0.3 or (not colour.is_valid() and state_tm < 0.5):
            return

        self.log_colour.append(colour.name)
        if colour.is_opposition() and not self.ignore_colour:
            if (
                self.catflap_active
                and not self.indexer.has_trapped_cargo
                and not self.indexer.has_cargo_in_chimney()
            ):
                self.next_state("trapping")
            else:
                self.next_state("clearing")
        else:  # our ball or ignore colour
            if self.catflap_active and self.ignore_colour:
                if self.indexer.has_trapped_cargo:
                    self.next_state("clearing")
                else:
                    self.next_state("trapping")
            else:
                # It is our ball so we have finished this process
                # The "stopped" state will work out if it needs to move the ball into the chimney
                self.next_state("stopped")

    @state(must_finish=True)
    def clearing(self, initial_call) -> None:
        if initial_call:
            if self.indexer.has_cargo_in_chimney():
                self.next_state("intake_clearing")
            else:
                self.shooter_control.clear()

        if not (
            self.shooter_control.current_state == "prepare_to_clear"
            or self.shooter_control.current_state == "clearing"
        ):
            self.next_state("stopped")

    @timed_state(duration=0.5, next_state="stopping", must_finish=True)
    def intake_clearing(self) -> None:
        self.indexer.run_tunnel_motor(Indexer.Direction.BACKWARDS)

    @timed_state(duration=0.5, next_state="stopping", must_finish=True)
    def forced_clearing(self) -> None:
        self.indexer.run_chimney_motor(Indexer.Direction.BACKWARDS)
        self.indexer.run_tunnel_motor(Indexer.Direction.BACKWARDS)

    @timed_state(duration=10.0, next_state="stopping", must_finish=True)
    def transferring_to_chimney(self) -> None:
        if self.indexer.has_cargo_in_chimney():
            self.next_state("stopping")
            return
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)
        self.indexer.run_tunnel_motor(Indexer.Direction.FORWARDS)

    @timed_state(duration=1.0, next_state="stopping", must_finish=True)
    def trapping(self, state_tm: float) -> None:
        self.indexer.open_cat_flap()
        if state_tm > 0.3:
            # Give some time for the piston to move the flap
            self.indexer.run_tunnel_motor(Indexer.Direction.FORWARDS)
            self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)
            self.indexer.has_trapped_cargo = True
