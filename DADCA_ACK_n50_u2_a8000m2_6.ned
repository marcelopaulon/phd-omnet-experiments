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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=839.45,7859.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1467.42,7863.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2865.59,7988.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3513.89,7820.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4086.91,7818.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4474.31,7921.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5576.62,7857.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=143.30,7647.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1926.75,7658.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2554.18,7708.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3179.05,7623.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4591.27,7522.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4996.16,7669.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1135.15,7398.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2159.64,7267.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3507.77,7467.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4169.66,7365.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5098.85,7264.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5362.20,7504.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5780.34,7365.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=150.97,7034.02");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=565.72,7227.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1362.59,7029.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1804.11,7207.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3237.38,7196.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3722.18,7170.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4454.80,7089.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5531.03,7060.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1040.74,6866.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2797.92,6925.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3142.72,6826.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3525.81,6764.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4008.41,6905.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4842.08,6975.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5954.86,6881.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=300.57,6590.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=714.16,6532.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1736.38,6659.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2135.17,6682.77");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4860.83,6550.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5420.27,6523.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1164.64,6493.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1570.29,6337.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2800.59,6362.57");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3427.41,6314.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3836.06,6329.92");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4356.08,6503.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5989.27,6466.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=207.15,6121.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1874.34,6071.48");
    }

}

