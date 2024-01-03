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
        @display("i=misc/sensor;p=322.19,7866.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=752.19,7949.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2202.06,7768.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3435.76,7979.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3946.64,7872.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4423.24,7922.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5668.51,7795.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=34.41,7574.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=562.48,7590.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1066.14,7618.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1559.59,7537.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2563.77,7564.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3032.17,7532.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4954.45,7577.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5339.66,7645.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=236.85,7270.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1926.51,7463.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3601.55,7388.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4247.69,7306.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4628.40,7322.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5267.81,7271.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5967.28,7437.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1159.53,7171.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2398.88,7194.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5809.56,7029.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=578.84,6934.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1619.71,6910.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3995.83,6994.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4945.97,6942.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5455.87,6794.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=935.56,6756.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1597.24,6521.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2148.61,6759.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3104.68,6728.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3961.17,6583.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4556.93,6709.10");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=148.04,6495.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=440.71,6278.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1989.40,6399.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2494.17,6434.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3389.11,6278.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4872.68,6276.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5666.71,6269.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1113.24,6088.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1781.63,6103.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2259.57,6025.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2957.16,6054.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4002.73,6077.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4463.48,6246.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=341.57,5926.32");
    }

}

