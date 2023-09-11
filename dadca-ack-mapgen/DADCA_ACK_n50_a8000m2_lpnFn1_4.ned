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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=58.72,30.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=154.58,7.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=219.47,45.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=282.20,127.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=162.16,82.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=186.85,148.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=242.05,209.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=6.61,100.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=62.82,169.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=134.65,218.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=313.99,23.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=373.34,96.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=387.80,28.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=192.00,248.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=130.63,314.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=230.09,320.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=36.95,301.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=67.99,231.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=443.39,98.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=513.84,56.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=574.32,60.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=674.13,105.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=544.00,122.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=628.99,157.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=679.80,205.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=675.82,274.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=602.87,275.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=551.94,213.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=503.67,254.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=525.66,315.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=670.07,352.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=741.71,365.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=789.05,422.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=724.06,450.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=806.95,487.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=805.19,342.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=769.06,282.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=761.99,194.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=741.86,72.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=814.64,93.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=826.95,255.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=921.98,257.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=961.27,312.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=995.71,379.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1054.60,317.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1120.46,287.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1207.24,218.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1128.80,159.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1033.89,151.41");
    }

}

