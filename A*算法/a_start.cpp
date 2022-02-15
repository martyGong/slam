#include <iostream>
#include <vector>
#include <queue>
#include<algorithm>
#include <fstream>
#include <sstream>
using namespace std;

vector<vector<int >> grid;
// vector<vector<int >> grid ={
// {0,0,1,1,1,0,0},
// {1,0,1,0,1,0,0},
// {0,0,0,1,1,0,0},
// {0,0,0,1,1,0,0},
// {0,0,0,1,1,0,0},
// {0,0,0,0,0,0,0}
// };

typedef struct{
    int index;
    int p_index;
}pathF;
typedef struct{
    int  index;
    int  weight;
    int  g ;
    int  h;
    int pindex;
}factor;

class a_start
{
        private:
        vector<factor>openList,closeList;
        int start_index,end_index;
        public:
            a_start(int start,int end);
            ~a_start();
             inline  int vtoi(int x, int y)
            {
                return x*grid[0].size() + y;
            }
            inline void itov(int i,int&x,int &y)
            {
                x = i/ grid[0].size();
                y = i % grid[0].size();
            }
           void process();
           void addNeighBor(factor v);
           void update(factor v);
           void addFactor(int x, int y,factor  v);
           bool isNeighBor(factor v1, factor v2);
           void vectorRm(factor f);
           void print();
};
     a_start::a_start(int start,int end):start_index(start),end_index(end)
     {

     }
    a_start::~a_start()
    {
    }
void a_start::print()
{
 #if 1
   //创建动态数组
   vector<vector<char>> vec(grid.size(),vector<char>(grid[0].size(),'-'));

   
  int currunt_index = end_index; 
  int x,y;
   bool  find = false;
  while(start_index != currunt_index)
  {
      
        for (auto f :closeList)
        {
            if(currunt_index == f.index)
            {
                itov(f.index,x,y);
                vec[x][y] = '+';
                currunt_index = f.pindex;
                //cout<< "currunt index "<< f.index  << "p inhdex  "<< currunt_index<<endl;
                find  = true;
                break;
            }
        }
        if(false == find )
        {
            cout << "can not find path "<<endl;
            break;
        }
  }

cout << "......."<<endl;
itov(start_index,x,y);
vec[x][y] = '+';

for(auto f:vec )
{
    for(auto v:f)
    {
        cout <<  v << " ";
    }
    cout << endl;
}
#endif

}   
void a_start::vectorRm(factor f)
{
   vector<factor>::iterator it;
   for(it = openList.begin();it!= openList.end();)
   {
       if (it->index == f.index)
       {
           it =  openList.erase(it);
       }
       else
       {
           it++;
       }
   }

}
bool a_start::isNeighBor(factor v1, factor v2)
{
    int  diff = abs(v1.index - v2.index );
    return ( diff == 1 ||  diff == grid[0].size());
}

void a_start::update(factor v)
{
    
    for(auto  f : openList)
    {
            if (isNeighBor(v,f))
            {
                if(f.g >  v.g+1)
                {
                    f.g = v.g+1;
                    f.weight = f.g + f.h;
                    f.pindex = v.index;
                }
            }
    }
}
void a_start::addFactor(int x, int y , factor v)
{
    if(x <0 || x >= grid.size() || y < 0 || y >= grid[0].size() )
    {
        return ;
    }
    if(closeList.end()  !=  find_if(closeList.begin(),closeList.end(),[&](factor &obj){return obj.index == vtoi(x,y);}))
    {
        return ;
    }
    if(grid[x][y] ==  1)
    {
        return ;
    }

    int end_x,end_y ;
    itov(end_index,end_x,end_y);
    factor v_temp;
    v_temp.index  = vtoi(x,y);
    v_temp.g = v.g+1;
    v_temp.h =  abs(x- end_x) + abs(y - end_y);
    v_temp.weight = v_temp.g +  v_temp.h;
    v_temp.pindex = v.index;
   //更新OPENLIST的权重
    update(v_temp);
    openList.push_back(v_temp);
}
//将邻居加入到ＯＰＥＮＬＩＳＴ
void a_start::addNeighBor(factor v)
{
    int x= 0,y = 0;
    int end_x = 0, end_y = 0;
    itov(v.index,x,y);
    itov(end_index,end_x,end_y);
    addFactor(x-1,y,v);
    addFactor(x+1,y,v);
    addFactor(x,y+1,v);
    addFactor(x,y-1,v);
}
void a_start::process()
{
    int x = 0 ,y = 0;
    int currunt_index=0;
   cout << "start index : "<< start_index << "    end  index :   " << end_index <<endl;
    itov(start_index,x,y);
    factor f;
    f.index = start_index;
    f.weight = 0 ;
    f.g = 0;
    f.h= 0;
    f.pindex = start_index;
   openList.push_back(f);
   while(!openList.empty())
    {
        factor f_temp;
        f_temp.index = start_index;
        f_temp.weight = 100000;
         //查找权重最小元素
          for(auto v : openList)
         {
              //  cout << "v_index "<< v.index  << "v_weight " << v.weight<<endl;
                if (v.weight  <  f_temp.weight)
                {
                     f_temp = v;
                }              
         }
         
          closeList.push_back(f_temp);
          if(end_index == f_temp.index) return ;


         // 将邻居加入到openlist
          addNeighBor(f_temp);
          vectorRm(f_temp);
         // cout << "end" <<endl;
        
    }
}


int main(int argc,char**argv)
{
  ifstream file("grid.txt",ios::in);
  if(file.is_open())
  {
      string line;
      while(getline(file,line))
      {
          vector <int >vec;
          stringstream ss;
          ss << line;

          while(!ss.eof())
          {
              int temp;
              while(ss >> temp)
              {
                  vec.push_back(temp);
              }
          }
          grid.push_back(vec);
      }
      file.close();
  }
  else
  {
      cout << "can not open file."<<endl;
      return  0;
  }
  int start ,end;
  cout << " please input the  start  index "<<endl;
  cin >> start;
  cout << "please input end index" <<endl;
  cin >> end;


   a_start  A(start, end);
    //cout << "process"<<endl;
    A.process();
 //   cout <<"print"<<endl;
    A.print();
    return 0;
}