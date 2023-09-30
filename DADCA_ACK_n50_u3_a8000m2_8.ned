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

network DADCA_ACK_n50_u3_a8000m2_8
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
        @display("i=misc/sensor;p=1495.21,7957.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2794.02,7897.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4056.22,7979.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5399.17,7918.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5913.74,7777.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=277.31,7679.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1937.32,7582.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2562.56,7607.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3597.50,7607.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4362.81,7727.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4937.71,7594.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5530.06,7520.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=368.68,7292.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=729.95,7357.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1320.61,7333.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2906.50,7287.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4071.96,7304.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5894.42,7270.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=101.94,7047.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=983.10,7018.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3267.50,7073.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4365.77,7076.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4796.49,7212.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5256.80,7035.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=359.44,6772.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1901.11,6967.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2325.31,6993.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3816.87,6818.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5605.26,6833.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1097.09,6639.41");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1638.11,6566.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2261.18,6608.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2923.09,6658.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3564.12,6567.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4633.40,6593.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5020.66,6572.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=626.40,6443.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5387.53,6365.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=262.02,6203.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1047.89,6249.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1432.14,6057.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2841.42,6232.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3357.77,6150.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3868.57,6229.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4901.42,6216.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5966.06,6104.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=156.63,5774.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=524.56,5921.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1064.04,5805.43");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1905.78,6004.31");
    }

}

