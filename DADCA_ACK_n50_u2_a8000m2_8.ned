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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=74.82,7930.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=516.88,7955.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=924.82,7827.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1361.90,7865.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2024.24,7886.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2383.40,7935.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2877.58,7808.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4277.04,7772.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2256.69,7588.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2614.80,7547.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3628.33,7750.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4832.52,7550.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5939.82,7634.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=479.84,7335.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1164.10,7439.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1698.58,7444.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4328.50,7387.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2045.68,7041.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2727.51,7026.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3171.40,7222.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4700.16,7063.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5302.70,7249.32");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5895.29,7219.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=126.22,6970.00");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=502.47,6853.77");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=880.17,6937.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2406.24,6869.10");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3754.78,6966.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4.16,6591.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1462.89,6575.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1821.29,6742.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3293.14,6683.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4249.90,6754.30");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4606.77,6557.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5091.16,6749.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5679.08,6746.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=443.15,6339.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1077.66,6367.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2135.48,6281.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3930.26,6410.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4293.64,6383.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5438.32,6471.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5889.01,6346.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=38.37,6022.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=729.67,6025.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1493.54,6215.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2792.31,6252.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3276.72,6118.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5045.61,6158.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1853.34,5828.72");
    }

}

