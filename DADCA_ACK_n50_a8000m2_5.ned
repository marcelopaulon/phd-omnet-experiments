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

network DADCA_ACK_n50_a8000m2_5
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
        @display("i=misc/sensor;p=19.40,7887.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1351.58,7782.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1862.89,7891.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3178.31,7946.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3578.96,7922.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4004.04,7971.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4714.84,7849.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=289.61,7605.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=629.75,7693.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2437.64,7513.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2752.10,7708.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3047.49,7511.54");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5100.83,7576.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5604.18,7712.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=817.92,7369.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1544.57,7356.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1904.27,7351.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3336.12,7261.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3755.56,7417.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4298.73,7498.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5348.30,7289.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=0.92,7225.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=416.23,7133.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1163.64,7229.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2637.29,7071.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4076.91,7107.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4590.78,7250.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5057.61,7042.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=979.69,6850.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1598.24,6994.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2011.48,6975.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=286.77,6558.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1398.12,6579.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2390.30,6745.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2994.09,6621.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3343.55,6752.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3837.33,6532.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4342.53,6563.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4820.95,6762.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5300.78,6672.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5751.96,6523.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1789.24,6417.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3360.21,6330.48");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4093.45,6280.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=801.91,6243.24");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1133.19,6065.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2461.43,6120.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2926.31,6235.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4876.73,6249.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5223.49,6140.91");
    }

}

