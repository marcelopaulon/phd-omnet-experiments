package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_a8000m2_lpnFn1_9
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=18.94,94.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=106.39,29.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=169.98,69.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=252.18,25.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=309.56,77.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=333.99,166.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=385.53,107.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=475.68,81.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=544.33,104.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=473.74,149.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=541.11,166.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=607.53,140.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=681.92,157.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=584.23,209.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=512.47,274.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=677.98,87.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=689.75,15.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=354.69,269.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=372.45,355.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=352.42,435.77");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=429.56,405.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=475.50,478.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=552.45,502.50");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=525.98,605.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=605.41,637.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=693.66,668.76");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=711.81,579.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=791.10,581.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=634.71,741.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=683.84,812.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=616.22,824.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=590.15,921.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=524.86,917.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=452.34,983.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=468.27,1045.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=424.96,1104.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=400.42,1027.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=430.11,1181.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=303.66,1069.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=367.04,1082.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=517.67,1177.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=587.39,1245.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=438.97,1298.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=451.23,1374.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=545.43,1349.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=635.27,1359.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=707.56,1415.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=610.08,1423.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=558.26,1461.35");
    }

}

