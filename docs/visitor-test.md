# First-visitor test (Phase 2 done criterion)

The roadmap says Phase 2 is done when "a first-time visitor understands what to
do within about 30 seconds". Only a real person can test that. This is a
15-minute script to run with 3 people who have never seen Atlas (ideally one
engineer outside robotics and one non-engineer).

## Setup

- Laptop with the live site open on the home page: https://djism.github.io/atlas/
  (fresh tab, scrolled to the top). A phone for one of the three sessions.
- A timer, and this page to write on. Do not explain anything first.

## Script (say only the words in quotes)

1. "This is a website about a robot project. Look at it as you would if a
   friend sent you the link. Think out loud." Start the timer.
2. Stop the timer at their first deliberate interaction with the robot or the
   lab (dragging furniture, dropping a box, moving a slider, switching a lab
   tab). Write the time. Do not help, even if they are stuck; note where.
3. After about 2 minutes: "In your own words, what is the blue ring? And the
   gray one?"
4. "Can you make the robot get lost?" Note whether they find the sensor range
   slider or move furniture, and how long it takes.
5. "Find the part with real car data. What does 0.43 % mean, roughly?"
6. "Was anything confusing, or did anything feel broken?"

## Record sheet

| | Person 1 | Person 2 | Person 3 |
| --- | --- | --- | --- |
| Background (engineer? robotics?) | | | |
| Device (laptop / phone) | | | |
| Seconds to first deliberate interaction | | | |
| Explained blue ring correctly (y/n, their words) | | | |
| Explained gray ring correctly | | | |
| Got the robot lost (y/n, how, seconds) | | | |
| Found the real-data section; read 0.43 % sensibly | | | |
| Confusions / bugs they hit | | | |

## Passing

- At least 2 of 3 interact deliberately within about 30 s.
- At least 2 of 3 explain the blue ring as "where the robot thinks it is"
  (or similar) without help.
- No one hits a bug that blocks them.

Anything that fails becomes the next polish task, with the person's own words
as the problem statement. Record the results (not names) in docs/progress.md.
