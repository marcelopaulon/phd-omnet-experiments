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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=27.00,153.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=87.79,190.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=194.30,245.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=237.16,320.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=171.74,331.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=102.21,357.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=21.04,315.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=81.53,252.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=1.00,81.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=61.67,407.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=114.95,471.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=190.91,425.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=298.37,233.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=314.39,310.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=383.94,326.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=313.39,407.21");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=383.71,468.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=412.12,563.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=470.12,507.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=471.87,595.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=554.49,494.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=294.88,496.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=327.34,551.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=238.57,549.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=496.29,396.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=630.82,567.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=695.10,553.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=699.30,620.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=657.94,471.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=812.27,665.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=771.47,610.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=769.57,515.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=842.76,476.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=802.98,391.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=771.43,338.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=733.62,423.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=594.11,401.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=604.11,332.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=543.54,280.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=584.46,214.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=521.07,188.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=449.66,197.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=417.88,261.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=706.10,285.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=841.54,571.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=928.48,599.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1008.63,587.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1033.29,642.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=950.95,669.13");
    }

}

