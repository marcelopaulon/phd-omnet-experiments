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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=373.12,7792.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1848.19,7828.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3665.29,7845.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4990.47,7809.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5448.11,7798.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=847.41,7701.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2176.57,7586.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2588.50,7626.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3202.80,7668.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4441.40,7752.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5785.51,7660.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=996.98,7347.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1384.40,7456.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1779.07,7328.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2365.81,7276.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2992.63,7368.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4087.09,7463.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4872.00,7456.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5403.79,7317.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=276.78,7211.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=658.22,7071.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3619.52,7201.97");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4533.90,7174.69");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1205.85,6934.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2439.78,6925.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3236.57,6916.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4212.87,6955.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4878.31,6922.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=51.14,6596.03");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=786.34,6630.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1600.46,6739.96");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2048.10,6695.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2738.49,6710.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3631.98,6747.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5058.11,6561.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5586.28,6685.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5959.82,6690.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=444.49,6389.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1111.65,6425.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2295.17,6335.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3038.78,6409.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3912.14,6373.58");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4365.96,6451.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1718.35,6099.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2630.51,6228.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3551.97,6192.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4181.77,6104.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4716.78,6106.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5301.86,6112.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5710.99,6216.34");
    }

}

