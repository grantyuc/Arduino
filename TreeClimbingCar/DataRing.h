#ifndef DataRing_H
#define DataRing_H

template<class T> class DataRing{
  public:
    DataRing() : size(10), last(0), ready(false) {data = new T[10];}
    DataRing(const int& n) : size(n), last(0), ready(false) {data = new T[n];}
    ~DataRing() {delete [] data;}

    void push(T d) {
      if(isFull()) {
        last = 0; 
        ready = true;
      }
      data[last] = d;
      ++last;
    }
    
    float average(){
      int sum = 0;
      for(int i=0; i<size; ++i)
        sum += data[i];
      float avg = float(sum)/size; 
      return avg;
    }
    
    float stdev(){
      float avg = average();
      float sqsum = 0;
      for(int i=0; i<size; ++i)
        sqsum += (data[i]-avg) * (data[i]-avg);
      float std = sqrt(sqsum/(size-1)); 
      return std;
    }

    void print(){
      for(int i=0; i<size; ++i){
        Serial.print(data[i]);
        Serial.print(" ");
      }
    }
    
    inline bool isReady() {return ready;}
    
  private:
    T* data;
    const int size;
    int last;
    bool ready;

    inline bool isFull() {return last == size;}
};

#endif // DataRing_H
