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

network DADCA_ACK_n50_u4_a8000m2_9
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
        @display("i=misc/sensor;p=478.96,7990.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1011.58,7881.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1499.29,7949.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2493.75,7977.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3053.45,7795.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4769.97,7945.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5089.28,7759.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5446.41,7944.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=39.71,7620.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=863.61,7510.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1892.59,7734.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2224.37,7526.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3878.38,7522.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4365.31,7683.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5843.79,7566.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1368.99,7286.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2883.99,7333.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3493.79,7397.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4634.74,7358.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5313.76,7435.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=334.16,7144.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2006.53,7159.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3386.07,7027.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4046.84,7035.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4980.16,7133.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5822.05,7045.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=867.52,6927.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1569.89,6970.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2494.40,6767.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5363.84,6968.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=30.44,6728.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1022.64,6552.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1548.34,6571.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2145.29,6636.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2961.08,6732.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4206.94,6575.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4579.65,6704.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5059.92,6690.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=641.50,6487.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2416.76,6336.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2798.23,6310.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3471.76,6301.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4825.46,6423.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5568.22,6493.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1128.16,6177.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1775.06,6121.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3869.70,6087.44");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4441.54,6216.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5153.96,6169.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=199.32,5946.09");
    }

}

