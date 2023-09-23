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

network DADCA_ACK_n50_u3_a8000m2_5
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
        @display("i=misc/sensor;p=948.81,7925.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1461.74,7765.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2088.19,7766.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2993.20,7824.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4170.93,7829.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4724.49,7966.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5058.30,7812.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=857.36,7555.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2462.50,7669.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3702.16,7583.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4869.10,7515.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5472.94,7744.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=359.63,7299.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1806.05,7453.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2342.47,7328.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3279.59,7387.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4265.95,7480.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=666.45,7031.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1153.18,7158.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1957.78,7079.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2749.54,7201.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3984.04,7183.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4471.24,7193.85");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5614.94,7236.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=331.37,6771.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1685.14,6851.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3157.62,6811.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4259.43,6781.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4702.77,6857.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5292.98,6785.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=975.33,6542.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2283.43,6660.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2822.42,6682.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3726.77,6746.58");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=358.01,6383.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1269.92,6350.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1944.36,6421.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3225.49,6345.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4214.74,6426.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4819.64,6321.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5513.32,6315.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=998.32,6052.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1834.00,6084.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2416.37,6226.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3713.49,6259.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4256.29,6076.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=129.46,5951.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=627.40,5798.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2237.16,5817.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2666.91,5935.69");
    }

}

