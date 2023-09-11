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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=81.50,7.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=44.13,69.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=153.83,57.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=111.57,194.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=52.64,223.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=182.31,122.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=182.51,193.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=256.99,164.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=208.30,263.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=294.58,303.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=328.54,238.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=389.90,297.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=264.68,226.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=250.27,351.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=323.42,392.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=350.91,480.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=251.88,479.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=222.00,406.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=396.07,363.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=436.55,473.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=454.56,415.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=161.20,386.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=438.38,250.03");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=332.95,130.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=263.77,96.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=449.69,39.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=366.95,51.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=404.02,139.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=487.15,144.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=563.33,146.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=501.74,222.20");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=547.22,309.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=607.53,366.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=609.87,295.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=589.99,227.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=690.11,216.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=726.04,157.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=671.74,99.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=645.47,25.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=714.05,23.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=762.35,98.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=836.83,103.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=899.85,167.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=963.95,161.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=964.93,223.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1019.18,252.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1024.70,121.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1074.64,84.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1082.24,154.12");
    }

}

