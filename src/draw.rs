
#[derive(Copy, Clone, Debug, Default)]
struct Sphere {
    rad:f64,
    center:Vec3,
    color:(f64,f64,f64),
}
impl Sphere{
    fn new(rad : f64, center : Vec3, color : (f64,f64,f64)) -> Self {
        Self { rad, center, color }
    }
}
#[derive(Copy, Clone, Debug, Default)]
struct Seg3 {
    p1:Vec3,
    p2:Vec3,
    color:(f64,f64,f64),
}
impl Seg3{
    fn new(p1 : Vec3, p2 : Vec3, color : (f64,f64,f64)) -> Self {
        Self { p1, p2, color }
    }
}

// impl Default for sphere{
//     fn default() -> Self {
//         Self {
//             rad: 0.0,
//             center:Vec3::default(),
//             color:(0,0,0),
//         }
//     }
// }
#[derive(Clone, Debug, Default)]
struct drawer {
    sphereList : Vec<Sphere>,
    lineList : Vec<Seg3>,
    debugTxt : String
}
impl drawer {
    fn createFinalString(&mut self) -> String {

        let mut finalRes = String::new();
        if CAN_DRAW {
            finalRes.push_str("[
            ");
            for i in 0..self.sphereList.len() {

                finalRes.push_str(&format!(" {{ \"Sphere\": {{
                    \"x\": {},
                    \"y\": {},
                    \"z\": {},
                    \"radius\": {},
                    \"r\": {},
                    \"g\": {},
                    \"b\": {},
                    \"a\": 0.5
                }}
            }}
            ",self.sphereList[i].center.x,self.sphereList[i].center.h,self.sphereList[i].center.y,self.sphereList[i].rad,self.sphereList[i].color.0,self.sphereList[i].color.1,self.sphereList[i].color.2)[..]);
            if i < self.sphereList.len() - 1 {
                finalRes.push_str(",");
            }
        }
        if self.sphereList.len() >= 1  && self.lineList.len() >= 1{
            finalRes.push_str(",");
        }

        for i in 0..self.lineList.len() {

            finalRes.push_str(&format!(" {{ \"Line\": {{
                \"x1\": {},
                \"y1\": {},
                \"z1\": {},
                \"x2\": {},
                \"y2\": {},
                \"z2\": {},
                \"width\": 1.0,
                \"r\": {},
                \"g\": {},
                \"b\": {},
                \"a\": 1.0
            }}
        }}
        ",self.lineList[i].p1.x,self.lineList[i].p1.h,self.lineList[i].p1.y,self.lineList[i].p2.x,self.lineList[i].p2.h,self.lineList[i].p2.y,self.lineList[i].color.0,self.lineList[i].color.1,self.lineList[i].color.2)[..]);
        if i < self.lineList.len() - 1 {
            finalRes.push_str(",");
        }
    }


    finalRes.push_str (&format!(",{{
        \"Text\": \"{} \"
    }}",self.debugTxt));

    
    finalRes.push_str ("
    ]");


    self.lineList.clear();
    self.sphereList.clear();
}
finalRes
}

fn draw(&mut self, pos : Vec3, rad : f64, color : (f64,f64,f64)) {
    self.sphereList.push(Sphere::new(rad,pos,color));
}

fn drawLine(&mut self, p1 : Vec3, p2 : Vec3, color : (f64,f64,f64)) {
    self.lineList.push(Seg3::new(p1,p2,color));
}

fn drawText(&mut self, input : String) {
    self.debugTxt = input;
}



}
