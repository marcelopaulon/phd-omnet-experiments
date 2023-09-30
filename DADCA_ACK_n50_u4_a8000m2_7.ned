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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=267.74,7820.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=632.88,7919.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=951.90,7764.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2205.89,7927.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2712.32,7968.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3199.44,7914.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5700.36,7809.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2356.26,7581.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3186.19,7537.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3542.21,7569.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3954.29,7599.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4402.15,7571.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5134.58,7709.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5907.90,7513.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=21.83,7396.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=527.85,7464.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1109.37,7321.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1685.72,7433.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2802.13,7347.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=846.69,7087.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3397.33,7226.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3858.97,7240.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4500.28,7114.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5262.32,7176.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5721.18,7036.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=399.58,6897.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1681.68,6904.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2199.46,6993.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2554.72,6922.37");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3539.22,6770.30");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4113.64,6773.29");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=31.10,6662.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=940.34,6715.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1326.21,6736.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5201.77,6710.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=22.00,6310.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=423.84,6325.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2470.82,6350.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3051.90,6488.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4060.61,6287.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4717.91,6417.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5400.65,6316.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=731.78,6076.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1180.83,6130.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1798.38,6119.73");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2721.28,6052.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3284.81,6144.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3696.74,6249.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5033.06,6253.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5825.18,6072.24");
    }

}

