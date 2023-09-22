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

network DADCA_ACK_n50_a8000m2_8
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
        @display("i=misc/sensor;p=89.26,7974.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=202.30,7982.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=371.99,7989.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=535.90,7972.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=688.97,7969.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=755.81,7976.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=825.80,7992.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=985.76,7978.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=254.60,7940.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=317.70,7939.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=449.51,7956.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=609.33,7947.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=928.53,7940.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=21.09,7893.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=92.78,7904.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=207.64,7896.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=387.12,7910.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=697.63,7889.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=784.21,7904.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=997.69,7873.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=62.21,7841.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=190.17,7833.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=288.86,7833.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=377.19,7849.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=484.68,7861.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=566.25,7868.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=824.64,7850.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=417.11,7802.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=569.20,7805.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=635.29,7817.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=901.91,7818.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=985.10,7809.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=49.19,7771.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=160.87,7742.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=270.10,7771.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=513.38,7768.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=698.53,7779.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=799.41,7748.40");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=64.46,7710.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=440.52,7710.28");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=727.58,7721.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=934.72,7735.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2.80,7691.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=251.37,7670.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=340.19,7697.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=399.98,7664.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=566.99,7693.56");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=701.97,7658.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=764.96,7668.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=844.43,7691.05");
    }

}

