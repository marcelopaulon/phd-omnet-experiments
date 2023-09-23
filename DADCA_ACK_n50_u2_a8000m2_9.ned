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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=742.72,7982.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1815.45,7878.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2297.43,7933.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4111.19,7894.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4458.28,7758.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5480.78,7808.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1298.89,7621.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2810.67,7650.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3743.78,7726.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4846.65,7741.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5235.05,7553.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=154.02,7456.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=901.75,7342.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2017.68,7357.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3401.76,7305.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4261.48,7314.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1535.50,7116.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2480.98,7066.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3129.33,7068.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3814.82,7192.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4781.00,7243.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5570.07,7060.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5944.55,7072.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=632.75,6987.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5068.31,6951.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=342.58,6647.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1338.31,6721.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1960.76,6652.77");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2941.14,6664.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3418.64,6646.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4133.52,6689.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4499.10,6651.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5544.38,6590.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5899.75,6721.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=915.48,6477.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1710.33,6388.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2445.80,6426.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3872.92,6379.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4962.92,6355.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5971.04,6312.25");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2933.41,6171.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3294.54,6226.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4396.56,6215.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5510.22,6094.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=355.89,5900.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=720.66,5873.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1348.60,5978.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2102.10,5942.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2473.17,5845.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3807.88,5988.15");
    }

}

