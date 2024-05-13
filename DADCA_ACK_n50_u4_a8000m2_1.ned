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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=58.26,7880.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=667.86,7999.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1300.14,7931.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1865.14,7851.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2255.99,7949.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2696.99,7946.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3240.91,7911.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3816.57,7778.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5723.06,7912.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1003.67,7711.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1733.85,7509.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2603.68,7526.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3009.45,7506.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4175.10,7741.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5455.54,7603.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=120.53,7400.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1372.16,7369.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2152.92,7307.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4093.24,7356.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5916.13,7432.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=520.14,7201.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1010.90,7145.35");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3232.48,7113.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3694.05,7179.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4269.64,7034.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4572.14,7229.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4955.59,7119.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=13.37,6811.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1535.31,6771.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2293.68,6813.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2669.91,6999.80");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5363.81,6911.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5757.16,6922.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=482.51,6523.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=903.31,6554.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1881.20,6642.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3086.73,6711.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3561.07,6725.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3997.76,6685.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4582.44,6738.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4946.64,6578.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=155.40,6281.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1429.70,6370.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2490.52,6503.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3032.59,6298.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3364.57,6431.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4413.85,6425.33");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5929.15,6347.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1009.09,6204.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1835.19,6234.50");
    }

}

