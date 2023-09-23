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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=1333.59,7863.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3376.11,7876.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4311.15,7866.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=485.14,7710.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=947.79,7730.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3628.22,7554.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3951.99,7749.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4571.03,7558.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4954.41,7557.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=260.27,7414.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=646.40,7365.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1625.66,7433.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2723.50,7306.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3300.33,7302.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4188.17,7353.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5397.66,7493.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1194.59,7153.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2094.89,7102.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3945.63,7016.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5902.70,7091.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=415.95,6771.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=742.14,6978.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1466.42,6823.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2730.46,6907.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3226.29,6908.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4547.41,6893.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5071.14,6979.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5455.88,6975.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=147.09,6520.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1131.85,6673.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1910.06,6680.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2649.28,6532.06");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3651.56,6516.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4335.39,6533.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4723.16,6531.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5676.87,6675.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=524.02,6356.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=992.45,6342.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1609.48,6436.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2282.54,6472.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3133.78,6427.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5345.33,6395.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=300.83,6041.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1524.37,6059.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2387.84,6135.70");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3803.30,6092.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4865.19,6188.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5889.27,6246.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=647.91,5797.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1804.33,5809.80");
    }

}

