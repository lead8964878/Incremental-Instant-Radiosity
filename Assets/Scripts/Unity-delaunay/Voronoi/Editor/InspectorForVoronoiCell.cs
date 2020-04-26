using UnityEngine;
using System.Collections;
using UnityEditor;
using VoronoiNS;

[CustomEditor(typeof(VoronoiCell))]
public class InspectorForVoronoiCell : Editor {

	public override void OnInspectorGUI()
	{
	VoronoiCell cell = target as VoronoiCell;
	
	if( cell.neighbours == null )
	{
	EditorGUILayout.HelpBox("Missing data: cell ought to have a Neighbours Dictionary", MessageType.Error);
	}
	else
	{
		EditorGUILayout.LabelField( cell.neighbours.Keys.Count+" neighbours..." );
	foreach( VoronoiCell otherCell in cell.neighbours.Values )
	{
	EditorGUILayout.BeginHorizontal();
	EditorGUILayout.LabelField("Neighbour:", otherCell.name );
	EditorGUILayout.EndHorizontal();
	}
	}
	}
}
