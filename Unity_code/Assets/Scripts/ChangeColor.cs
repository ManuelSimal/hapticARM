using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeColor : MonoBehaviour {
    Renderer cubeRenderer;
    private Color defaultColor;
    public Color color;

    // Use this for initialization
    void Start () {
        //Get the Renderer component from the new cube
        cubeRenderer = this.GetComponent<Renderer>();
        defaultColor = new Color(0, 0, 1, 0);
        cubeRenderer.material.SetColor("_Color", defaultColor);
        color = defaultColor;
    }
	
	// Update is called once per frame
	void Update () {
        //Call SetColor using the shader property name "_Color"
        cubeRenderer.material.SetColor("_Color", color);
    }
}
