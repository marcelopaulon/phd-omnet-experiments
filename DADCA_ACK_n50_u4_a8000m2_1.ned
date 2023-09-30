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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=2213.96,7954.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3321.65,7856.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3685.81,7825.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4175.56,7944.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4593.77,7793.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5231.12,7840.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1354.33,7579.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1952.34,7572.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2624.03,7721.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5753.82,7655.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=234.17,7331.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=911.87,7333.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2481.77,7382.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2886.65,7336.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3738.53,7458.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4070.30,7338.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4950.79,7366.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5466.40,7429.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1757.47,7034.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2063.51,7239.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3474.20,7172.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4585.91,7088.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5745.38,7045.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=7.81,6878.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=692.07,6924.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1178.83,6969.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3155.78,6968.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4026.53,6894.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4966.87,6784.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5287.21,6999.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1068.34,6559.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1533.10,6651.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2132.99,6604.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2500.78,6760.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2944.57,6681.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4615.10,6561.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=459.79,6415.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3603.08,6358.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3971.24,6498.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5181.50,6337.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5678.30,6311.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=823.94,6076.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2046.47,6203.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4880.04,6149.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2114.21,5800.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2762.46,5825.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3087.26,5969.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3985.06,5972.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4341.54,5958.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5047.23,5785.77");
    }

}

