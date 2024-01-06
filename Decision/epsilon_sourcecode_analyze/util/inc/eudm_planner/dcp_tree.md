类 DcpTree:
横向动作：LaneKeeping、LaneChangeLeft、LaneChangeRight
纵向动作：Maintain、Acc、Dec
```C
DcpAction{
    lon;
    lat;
    t;
    DcpAction(lon,lat,t);
}
```
类成员：
1. tree_height = 5.0
2. layer_time = 1.0
3. last_layer_time = 1.0
4. DcpAciton ongoing_action
5. vector<vector<DcpAction>> action_script;

该类的作用主要就是利用ongoing_action来生成dcp_tree

主要成员函数：
ErrorType GenerateActionScript():
dcp树的生成依赖如下：
1. 在横向上，我们最多允许一次更改横向动作，纵向上一直保持某一种状态。
比如，当前动作是 <keep,maintain>,深度为3，那所有可能的序列为：  
<keep,maintain>-><keep,maintain>-><keep,maintain>  
<keep,maintain>-><left,maintain>-><left,maintain>  
<keep,maintain>-><keep,maintain>-><left,maintain>  
<keep,maintain>-><right,maintain>-><right,maintain>  
<keep,maintain>-><keep,maintain>-><right,maintain>  
<keep,acc>-><keep,acc>-><keep,acc>  
<keep,acc>-><left,acc>-><left,acc>  
<keep,acc>-><keep,acc>-><left,acc>  
<keep,acc>-><right,acc>-><right,acc>  
<keep,acc>-><keep,acc>-><right,acc>  
<keep,dec>-><keep,dec>-><keep,dec>  
<keep,dec>-><left,dec>-><left,dec>  
<keep,dec>-><keep,dec>-><left,dec>  
<keep,dec>-><right,dec>-><right,dec>  
<keep,dec>-><keep,dec>-><right,dec>  
