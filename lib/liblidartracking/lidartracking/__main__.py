"""Headless synthetic association demo and JSONL replay (no hardware writes)."""
import argparse
from dataclasses import asdict
import json
from .core import Calibration, CameraFrame, Detection, RangeSample, ServoFeedback, VehicleRanger


def synthetic():
    calibration = Calibration(1456,1088,1200.,1200.,728.,544.,
                              (1.,0.,0.,0.,1.,0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.),.001,True)
    ranger = VehicleRanger(calibration)
    ranger.select(7)
    for index in range(20):
        stamp = 1+index*.05
        frame = CameraFrame(stamp,.002,1456,1088,(Detection(7,(670,495,786,593),.95),))
        sample = RangeSample(index,stamp,.005,80-index*.25,80-index*.25,25,25,True)
        servo = ServoFeedback(stamp+.01,0.,.001,True,0.,stamp+.01)
        if index == 10:
            # Demonstrates invalidation rather than carrying old distance forward.
            sample = RangeSample(index,stamp,.005,-.1,-.1,0,0,True)
        yield ranger.update(frame,sample,servo,stamp+.01)


def replay(config, source):
    with open(config, encoding="utf-8") as file:
        ranger = VehicleRanger(Calibration(**json.load(file)))
    with open(source, encoding="utf-8") as file:
        # readline bound avoids allocating an arbitrarily long malformed record.
        for line in iter(lambda: file.readline(65537), ""):
            if len(line) > 65536:
                raise ValueError("replay event too large")
            event = json.loads(line)
            ranger.select(event["selected_id"])
            image = event["camera"]
            image["detections"] = tuple(Detection(**item) for item in image["detections"])
            servo = ServoFeedback(**event["servo"]) if event.get("servo") else None
            yield ranger.update(CameraFrame(**image),RangeSample(**event["range"]),servo,event["now_s"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--synthetic",action="store_true")
    parser.add_argument("--calibration")
    parser.add_argument("--replay")
    args = parser.parse_args()
    if args.synthetic and not (args.calibration or args.replay):
        results = synthetic()
    elif args.calibration and args.replay and not args.synthetic:
        results = replay(args.calibration,args.replay)
    else:
        parser.error("use --synthetic OR --calibration FILE --replay EVENTS.jsonl")
    for result in results:
        print(json.dumps(asdict(result),allow_nan=False))


if __name__ == "__main__":
    main()
