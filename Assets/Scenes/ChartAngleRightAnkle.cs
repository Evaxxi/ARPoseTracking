using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using XCharts.Runtime;

public class XChartRightAnkle : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        var chart = gameObject.GetComponent<LineChart>();
        if (chart == null)
        {
        chart = gameObject.AddComponent<LineChart>();
        chart.Init();
        }
        chart.SetSize(580, 300);
        var title = chart.EnsureChartComponent<Title>();
        title.text = "RightAnkle";
        var tooltip = chart.EnsureChartComponent<Tooltip>();
        tooltip.show = true;

        var legend = chart.EnsureChartComponent<Legend>();
        legend.show = false;
         var xAxis = chart.EnsureChartComponent<XAxis>();
        xAxis.splitNumber = 10;
        xAxis.boundaryGap = true;
        xAxis.type = Axis.AxisType.Category;

        var yAxis = chart.EnsureChartComponent<YAxis>();
        yAxis.type = Axis.AxisType.Value;
        chart.RemoveData();
        chart.AddSerie<Line>("line");
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
