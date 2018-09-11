//
//  ArriveViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/04.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

class ArriveViewController: UIViewController {

        var b_Info:ReceptionViewController.bookInfoJson?
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    @IBAction func arriveButtonAction(_ sender: Any) {
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Door") as! DoorViewController
        nextView.b_Info = self.b_Info!
        self.present(nextView, animated: true, completion: nil)
    }
    
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destinationViewController.
        // Pass the selected object to the new view controller.
    }
    */

}
