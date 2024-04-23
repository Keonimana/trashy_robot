import time 
LIMIT = 100 
class TimerStream(): 
    def __init__(self): 
        self.baselines = {} 
        self.streams = {}
    def set_baseline(self,streamName): 
        self.baselines[streamName] = time.time() 
    def add_stream(self,name): 
        self.baselines[name] = time.time() 
        self.streams[name] = [] 
    def add_time(self, streamName): 
        if streamName not in self.baselines.keys() or streamName not in self.streams.keys(): 
            print("Missing Stream Def") 
            return 
        end = time.time() 
        if len(self.streams[streamName]) > LIMIT: 
            self.streams[streamName].pop(0)
        self.streams[streamName].append(end - self.baselines[streamName]) 
        self.baselines[streamName] = end 
    def get_average_time(self, streamName): 
        stream = self.streams[streamName] 
        return sum(stream) / len(stream)
    def get_max_time(self,streamName): 
        return max(self.streams[streamName]) 
    def get_min_time(self,streamName): 
        return min(self.streams[streamName]) 
    def format_streamPrint(self,streamName): 
        avg = self.get_average_time(streamName) 
        peak = self.get_max_time(streamName) 
        small = self.get_min_time(streamName) 
        print(f'{streamName}: ({avg:.3f}s, {peak:.3f}s, {small:.3f}s)')




