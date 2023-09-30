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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=39.14,7777.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=475.44,7842.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1135.06,7974.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3208.43,7897.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4722.53,7798.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=898.64,7702.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1633.43,7676.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2855.25,7749.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3516.10,7725.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4184.12,7555.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5100.66,7596.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5560.47,7678.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=26.16,7371.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=696.33,7321.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1294.45,7270.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1681.26,7273.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2276.18,7441.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3081.61,7346.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=334.35,7157.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2608.04,7148.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3538.75,7184.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4030.57,7151.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4782.71,7229.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5339.61,7078.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=318.76,6791.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1869.70,6948.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2923.07,6912.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3749.51,6812.43");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=977.29,6660.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1440.24,6627.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2650.96,6686.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3311.05,6714.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4303.20,6639.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5141.55,6561.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=517.39,6418.94");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2206.76,6470.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3934.35,6407.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4722.43,6361.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=153.61,6109.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=806.13,6069.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1169.83,6255.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1556.37,6235.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2451.77,6083.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2899.33,6197.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3377.89,6180.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4397.80,6185.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2901.51,5788.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3284.53,5806.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3893.33,5907.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4284.33,5849.82");
    }

}

