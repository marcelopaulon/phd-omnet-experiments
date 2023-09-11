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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=176.52,103.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=194.29,166.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=128.46,240.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=97.56,304.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=7.00,266.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=36.24,363.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=70.88,199.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=7.59,177.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=48.11,67.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=198.91,350.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=241.47,414.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=219.77,497.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=161.44,572.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=150.26,499.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=176.08,423.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=125.90,154.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=278.20,349.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=270.75,242.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=330.49,298.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=352.32,240.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=356.18,170.32");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=394.59,113.80");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=381.26,30.31");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=447.25,13.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=294.72,72.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=223.80,44.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=267.82,175.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=141.51,53.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=97.54,408.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=527.14,54.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=589.95,103.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=332.38,513.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=391.65,526.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=451.87,505.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=475.55,450.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=495.03,382.85");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=578.11,386.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=557.30,442.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=649.68,329.91");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=714.93,256.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=656.64,217.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=582.84,195.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=506.78,226.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=573.40,323.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=513.75,537.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=575.73,518.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=484.58,318.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=412.75,382.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=349.29,426.96");
    }

}

