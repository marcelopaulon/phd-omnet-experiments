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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=35.81,97.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=67.23,25.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=0.27,27.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=79.78,212.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=163.29,177.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=217.26,125.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=290.58,74.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=131.79,249.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=158.99,84.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=36.52,263.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=17.63,172.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=179.33,286.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=173.07,353.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=237.57,303.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=285.87,433.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=270.68,514.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=207.52,490.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=322.07,546.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=401.26,500.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=403.00,571.43");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=465.64,592.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=433.42,660.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=489.40,711.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=557.03,666.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=552.00,729.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=648.09,730.46");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=729.10,724.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=701.77,818.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=799.52,793.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=819.42,871.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=752.72,856.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=738.11,972.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=688.05,911.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=855.06,767.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=896.63,886.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=937.98,798.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1010.92,861.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1068.09,824.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1094.79,883.38");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1186.78,872.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1193.66,796.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1029.92,981.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=939.92,941.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=932.93,1021.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=987.97,1090.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1000.99,1158.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=903.24,1219.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=902.72,1307.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=885.75,1374.35");
    }

}

