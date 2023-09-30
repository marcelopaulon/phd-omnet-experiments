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
        @display("i=misc/sensor;p=216.50,7967.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=617.77,7848.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2331.33,7788.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3430.95,7801.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3860.95,7983.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4223.26,7971.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5319.84,7973.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5823.47,7796.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1054.02,7717.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1515.49,7695.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1861.45,7748.47");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2736.58,7715.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4419.20,7593.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4869.50,7651.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=142.10,7404.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1284.20,7408.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2417.67,7362.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3703.47,7503.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5360.67,7329.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=370.64,7069.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1537.94,7162.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1970.48,7212.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2613.69,7029.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3122.94,7196.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3739.84,7140.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4197.71,7154.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4788.70,7178.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5911.80,7210.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=917.26,6960.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1381.25,6810.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1772.93,6848.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2226.51,6902.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3852.03,6788.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5042.79,6866.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5641.21,6934.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=262.27,6702.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1110.17,6530.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2140.46,6537.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3330.03,6740.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4690.99,6673.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5650.22,6577.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=734.33,6502.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1699.89,6365.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2787.48,6454.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4341.97,6279.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5046.56,6349.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5949.13,6346.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=685.35,6042.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1266.13,6213.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2478.75,6259.23");
    }

}

