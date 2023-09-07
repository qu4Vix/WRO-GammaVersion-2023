using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovimientoCoche : MonoBehaviour
{
    public long posX = 1000;
    public long posY = -1200;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(posX/1000.0f, transform.position.y, posY/1000.0f);
    }
}
