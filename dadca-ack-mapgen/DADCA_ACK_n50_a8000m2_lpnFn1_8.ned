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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=186.04,131.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=122.89,191.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=265.47,134.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=316.19,89.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=23.00,76.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=161.15,19.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=218.82,78.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=404.89,126.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=450.68,211.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=389.93,236.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=519.68,135.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=589.64,119.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=459.03,51.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=564.03,195.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=324.48,266.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=557.98,66.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=614.36,29.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=653.49,162.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=624.96,228.09");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=597.56,299.30");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=478.16,298.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=308.36,181.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=377.35,40.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=289.36,23.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=261.39,262.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=222.71,193.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=51.04,181.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=50.41,268.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=142.37,355.56");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=86.81,320.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=146.21,255.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=75.79,33.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=16.82,11.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=329.52,368.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=258.76,394.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=271.71,456.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=204.94,339.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=65.06,381.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=21.71,427.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=393.81,347.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=542.72,369.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=698.17,103.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=702.44,200.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=806.05,180.24");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=874.11,122.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=881.71,57.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=844.86,238.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=974.97,26.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=984.67,132.78");
    }

}

