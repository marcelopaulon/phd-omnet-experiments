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

network DADCA_ACK_n50_u8_a8000m2_2
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
        @display("i=misc/sensor;p=1166.93,7843.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1694.82,7760.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2145.25,7777.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2638.97,7856.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3107.21,7825.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3739.47,7836.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4672.16,7777.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5771.67,7798.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2855.52,7556.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4080.96,7667.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=267.02,7286.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1565.42,7326.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2060.39,7431.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3436.19,7310.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4252.84,7272.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5652.77,7438.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=673.92,7083.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1129.54,7065.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2500.16,7149.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3034.55,7028.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3797.30,7191.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5994.36,7109.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2298.51,6792.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4322.10,6792.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4821.22,6918.23");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5229.83,6963.55");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=100.60,6657.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1817.59,6711.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2834.12,6715.62");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3328.88,6692.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3936.66,6664.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5067.66,6570.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5854.19,6682.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=896.88,6510.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1391.46,6345.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2116.52,6334.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2628.58,6386.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3669.29,6405.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4523.02,6341.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=335.73,6180.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=837.08,6140.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2987.39,6262.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3349.71,6082.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5003.25,6187.96");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5579.67,6150.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=33.77,5821.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1259.18,5932.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1979.39,5874.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2670.74,6014.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4008.13,6017.72");
    }

}

