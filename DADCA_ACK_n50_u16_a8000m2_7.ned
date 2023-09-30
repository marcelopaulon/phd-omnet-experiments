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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=635.28,7796.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2313.30,7931.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3152.72,7954.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3752.18,7756.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4137.33,7885.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5361.49,7828.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1273.46,7662.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2652.38,7739.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2942.64,7543.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3399.15,7602.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4644.71,7704.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5086.40,7512.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5703.53,7518.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=490.65,7468.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1688.32,7435.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2163.84,7331.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2650.85,7331.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4225.05,7441.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1077.57,7193.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1759.80,7070.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3778.08,7039.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4568.50,7078.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=408.28,6765.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1432.23,6804.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2093.92,6930.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3258.54,6941.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4980.20,7007.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5395.02,6975.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2450.47,6616.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3405.40,6533.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3749.93,6643.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4290.06,6593.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4689.08,6686.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5461.34,6551.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=852.25,6363.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1988.24,6440.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2942.83,6440.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4507.90,6269.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5100.43,6475.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5900.52,6507.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=576.98,6093.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1544.52,6025.07");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2480.53,6054.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3143.72,6027.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3842.33,6261.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4918.81,6143.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=110.56,5901.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1097.30,5945.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1901.31,5794.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3611.18,5879.51");
    }

}

