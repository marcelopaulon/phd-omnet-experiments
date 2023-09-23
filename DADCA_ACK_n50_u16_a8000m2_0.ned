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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=408.16,7786.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1243.42,7895.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3002.29,7983.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3693.34,7975.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4364.84,7903.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4800.41,7900.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=756.13,7701.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1609.94,7677.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2251.70,7743.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2721.27,7710.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3613.32,7573.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5336.42,7716.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5884.39,7682.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=919.91,7389.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2910.35,7257.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3248.36,7371.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4115.60,7467.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4699.31,7474.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5334.21,7320.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=36.31,7048.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=600.59,7125.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1841.89,7210.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2277.81,7141.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3730.52,7034.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4172.58,7068.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4674.38,7091.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5684.15,7143.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1094.36,6988.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2791.79,6801.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3278.13,6912.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=619.94,6732.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=985.14,6634.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1572.95,6683.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2262.28,6734.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4101.64,6610.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4877.67,6722.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5399.11,6687.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5972.48,6622.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=326.88,6396.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=731.96,6315.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1888.84,6410.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2320.39,6360.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2874.79,6361.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3712.90,6496.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4396.42,6383.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1178.11,6109.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1696.65,6035.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3212.39,6052.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4923.04,6185.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5650.44,6141.96");
    }

}

