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

network DADCA_ACK_n50_a8000m2_4
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
        @display("i=misc/sensor;p=632.15,7988.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1025.07,7983.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1496.34,7933.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1906.51,7787.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3941.54,7863.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5004.27,7815.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=232.85,7576.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1084.65,7576.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1473.87,7483.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2409.74,7584.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2795.37,7555.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4667.92,7668.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5357.53,7638.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5816.08,7560.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=826.64,7233.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3693.72,7467.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4181.07,7455.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=269.78,6969.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1491.12,7106.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1895.14,7178.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2526.23,7124.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3199.67,7203.99");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4923.20,7208.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5476.88,7127.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5931.12,7050.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=639.99,6768.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1002.58,6877.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3218.88,6760.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3742.93,6951.19");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4424.89,6910.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=942.94,6507.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1966.47,6668.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3868.45,6481.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4293.73,6542.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4857.81,6694.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5548.12,6692.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=192.04,6257.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2620.41,6391.36");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3304.89,6367.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4825.79,6327.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=847.96,6057.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1329.90,6055.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1903.98,6051.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2280.38,6118.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2897.36,6125.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3420.27,5944.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3920.12,5961.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=16.31,5710.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=501.08,5739.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4487.66,5923.91");
    }

}

