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

network DADCA_ACK_n50_u3_a8000m2_5
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
        @display("i=misc/sensor;p=1327.87,7806.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2171.71,7921.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3461.33,7964.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3836.56,7906.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5133.58,7835.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=62.74,7522.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=368.99,7704.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1787.85,7600.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2238.32,7541.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2935.64,7748.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3624.08,7628.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4175.82,7715.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4830.28,7595.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5592.22,7619.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=904.52,7456.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3057.79,7271.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4316.53,7372.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=405.59,7090.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1166.55,7106.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1566.94,7152.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1926.44,7114.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2518.03,7020.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3952.33,7191.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4688.22,7062.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5064.44,7163.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5626.63,7185.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2234.02,6773.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2845.37,6797.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3258.79,6867.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3633.86,6951.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4124.19,6884.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5790.72,6844.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=359.80,6723.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1003.42,6731.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1501.25,6714.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3950.55,6545.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4519.10,6635.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5267.91,6656.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=430.41,6281.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=811.33,6404.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1218.73,6391.94");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1710.26,6383.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2627.49,6342.01");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3306.30,6502.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5367.68,6280.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5742.60,6497.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1238.44,6040.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2128.19,6252.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2864.81,6051.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4132.06,6197.66");
    }

}

