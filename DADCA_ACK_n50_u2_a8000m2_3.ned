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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=707.82,7807.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1346.41,7997.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2197.04,7839.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3795.95,7883.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4199.94,7761.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4905.85,7872.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=195.77,7688.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2604.09,7534.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3277.32,7641.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4897.95,7511.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5480.05,7654.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=504.04,7290.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1180.10,7391.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1587.22,7352.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2068.59,7310.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3665.52,7386.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5950.46,7494.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1800.02,7071.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3057.42,7044.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3469.77,7089.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4612.41,7249.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5356.38,7236.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5799.37,7086.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=472.98,6853.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=820.03,6910.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1328.44,6882.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2053.02,6780.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2467.46,6939.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3974.94,6882.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4476.17,6800.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5156.81,6854.23");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2651.83,6576.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3065.00,6658.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4059.17,6528.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5715.77,6594.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=473.44,6430.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1050.84,6461.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1809.31,6356.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2286.85,6405.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3712.95,6442.77");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4685.58,6461.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5057.32,6453.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=75.12,6132.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=621.92,6031.72");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1324.31,6048.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2952.16,6217.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3531.54,6033.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4208.38,6029.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4854.91,6049.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5597.22,6055.71");
    }

}

