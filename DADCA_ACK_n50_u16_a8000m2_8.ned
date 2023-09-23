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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=13.28,7989.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=531.64,7978.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=884.83,7907.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1767.17,7897.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2436.51,7980.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2784.17,7935.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3276.53,7868.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4391.40,7908.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5226.13,7889.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=251.22,7664.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1243.91,7701.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2257.83,7608.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2607.40,7541.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3817.74,7737.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5373.12,7543.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5696.92,7749.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=867.18,7371.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1859.58,7391.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3323.14,7492.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3637.48,7272.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4071.32,7262.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4608.68,7476.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4908.63,7266.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=10.84,7201.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=416.68,7123.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2751.98,7212.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1186.38,6998.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2162.54,6949.49");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3248.19,6940.94");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4520.31,6857.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4982.87,6817.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5453.11,6957.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=284.44,6667.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=781.32,6710.56");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1927.12,6588.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2628.69,6606.28");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3500.11,6636.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5599.23,6597.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=515.66,6309.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=881.88,6293.56");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1299.36,6374.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3227.51,6329.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3832.90,6462.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4350.96,6310.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4717.32,6478.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2191.61,6237.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3827.17,6073.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5152.11,6168.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1301.86,5904.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1740.41,5996.36");
    }

}

