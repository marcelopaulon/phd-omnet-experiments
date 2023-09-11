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
        @display("i=misc/sensor,p=89.75,167.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=164.25,34.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=69.38,40.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=12.77,135.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=194.89,138.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=174.59,233.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=264.79,205.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=286.49,270.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=330.92,311.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=364.58,369.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=352.71,454.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=422.19,411.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=450.65,317.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=526.60,343.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=541.38,427.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=478.80,465.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=522.13,546.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=437.91,553.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=592.98,481.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=620.46,406.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=615.54,312.85");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=666.73,268.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=719.70,194.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=770.67,233.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=698.35,368.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=686.00,446.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=778.75,447.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=845.40,342.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=784.67,367.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=854.38,403.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=860.73,490.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=789.86,556.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=653.67,540.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=683.15,141.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=759.37,83.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=764.20,12.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=819.84,90.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=873.62,124.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=519.08,268.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=465.77,232.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=467.45,143.25");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=559.80,135.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=620.52,100.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=865.85,2.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=624.64,197.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=942.57,43.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1006.73,71.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=909.67,190.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1005.91,198.29");
    }

}

