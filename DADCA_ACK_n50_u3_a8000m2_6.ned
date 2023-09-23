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

network DADCA_ACK_n50_u3_a8000m2_6
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
        @display("i=misc/sensor;p=256.79,7804.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2114.41,7948.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3035.82,7962.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4481.86,7906.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4874.77,7945.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=596.25,7597.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1896.62,7674.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2716.14,7567.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3465.20,7732.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3844.00,7661.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4184.32,7546.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=212.57,7438.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1526.56,7419.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1937.18,7305.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2345.55,7444.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4708.08,7462.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=502.46,7171.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1157.43,7186.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2515.56,7104.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2996.12,7254.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3449.23,7143.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3873.68,7067.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4370.72,7049.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1819.30,6871.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3150.55,6764.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4978.33,6937.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5648.62,6884.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=242.80,6716.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=875.83,6736.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2124.64,6554.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2619.46,6731.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3511.87,6698.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4129.77,6567.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1474.01,6398.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2796.22,6348.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3479.17,6311.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4751.06,6307.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5254.80,6311.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5943.23,6290.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=126.57,6256.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=553.23,6236.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1055.66,6084.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1777.94,6205.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2442.74,6196.74");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3711.01,6044.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2026.10,5804.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4376.57,5907.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4908.75,5932.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5748.89,5900.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=9.08,5736.43");
    }

}

