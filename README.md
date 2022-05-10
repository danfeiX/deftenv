# DeftEnv: A Simulated Environment for Sequential Manipulation

This is a reference implementation for the environments used in [Deep Affordance Foresight](https://arxiv.org/abs/2011.08424).
If you find this codebase useful, please cite the work using the following bibtex:

```
@inproceedings{xu2021deep,
  title={Deep affordance foresight: Planning through what can be done in the future},
  author={Xu, Danfei and Mandlekar, Ajay and Mart{\'\i}n-Mart{\'\i}n, Roberto and Zhu, Yuke and Savarese, Silvio and Fei-Fei, Li},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={6206--6213},
  year={2021},
  organization={IEEE}
}
```

## Getting Started
###1. Install DeftEnv:
```
$ git clone https://github.com/danfeiX/deftenv.git
$ cd deftenv
# you may want to create a separate virtualenv to avoid package contamination
$ pip install -e deftenv
```

### 2. Run an example
Run and visualize a guided exploration session and collect demonstrations.

Tool manipulation + stacking:
```angular2html
$ python scripts/kitchen_main.py --mode demo --env ToolStackAP --file demo.hdf5 --gui --keep_failed_demos --keep_interrupted_demos
```

Kitchen make coffee or tea (random sample task goal):
```angular2html
$ python scripts/kitchen_main.py --mode demo --env KitchenDualCoffeeAP --file demo.hdf5 --gui --keep_failed_demos --keep_interrupted_demos
```

### 3. Extract training data
Extract training data for the demonstrations that we just collected in `demo.hdf5`
```angular2html
python deftenv/scripts/kitchen_main.py --mode extract_skill --file demo.hdf5 --extract_name training_data.hdf5
```