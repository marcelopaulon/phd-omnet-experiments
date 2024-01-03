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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=2719.84,7851.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3478.99,7946.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4044.46,7902.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4863.38,7860.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=893.32,7717.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1485.52,7734.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2003.20,7697.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3250.37,7566.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4316.13,7511.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5324.61,7645.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1448.57,7353.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2404.42,7360.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2793.76,7444.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3928.10,7292.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=413.67,7109.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1086.58,7244.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1794.87,7187.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3350.10,7175.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4202.56,7052.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4912.44,7187.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5397.18,7164.65");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5987.71,7125.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1556.32,6877.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2471.91,6997.81");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2978.22,6843.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=798.20,6712.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1156.46,6673.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1994.67,6736.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3685.55,6607.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4383.73,6740.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4739.82,6644.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5122.40,6582.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5682.24,6567.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=352.37,6504.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3143.76,6444.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4001.70,6316.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=439.81,6132.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1123.90,6250.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1605.42,6239.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2012.25,6130.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2789.42,6244.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3506.27,6054.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4521.51,6243.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5062.52,6057.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5416.39,6245.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=900.70,5854.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3059.39,5900.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5568.89,5916.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1488.42,5657.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1872.24,5686.22");
    }

}

