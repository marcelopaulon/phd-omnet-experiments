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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=558.23,7929.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1322.53,7800.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2330.79,7845.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2735.47,7839.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3967.58,7774.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4403.04,7866.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4748.54,7761.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5109.59,7807.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=297.09,7686.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=867.35,7646.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1953.28,7626.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=571.52,7459.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2179.53,7350.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2533.33,7301.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3417.84,7413.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4072.91,7368.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4736.26,7377.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=26.71,7131.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=479.42,7072.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=925.12,7132.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1547.47,7179.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2994.89,7081.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4327.28,7023.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5053.01,7220.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5446.21,7234.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1186.91,6798.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1932.46,6932.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3295.20,6784.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3628.58,6921.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4876.28,6813.08");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5493.02,6873.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=159.63,6687.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=813.52,6716.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2356.74,6749.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4015.80,6722.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4600.69,6525.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=457.33,6419.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1539.62,6466.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1912.87,6307.78");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2850.33,6445.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3546.63,6315.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4068.41,6375.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=87.72,6218.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=941.15,6249.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2507.92,6201.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5054.29,6141.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5921.30,6135.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=678.94,5999.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1340.88,5952.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2034.14,5794.08");
    }

}

