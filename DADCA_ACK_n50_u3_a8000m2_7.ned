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

network DADCA_ACK_n50_u3_a8000m2_7
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
        @display("i=misc/sensor;p=30.50,7788.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=610.31,7888.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3306.52,7807.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5232.95,7856.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1043.99,7546.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1486.95,7664.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2268.15,7581.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2797.58,7653.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3975.15,7672.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5761.29,7556.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=242.89,7465.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4450.25,7334.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4922.64,7377.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=892.00,7207.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1503.40,7040.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2092.09,7072.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2811.59,7023.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3301.18,7181.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4093.50,7057.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=268.17,6812.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3761.30,6908.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4906.85,6992.85");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=594.33,6604.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1780.11,6599.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2301.67,6689.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3433.04,6687.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4403.92,6670.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5378.59,6623.14");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1067.42,6496.19");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2767.36,6476.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4736.20,6385.68");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5978.25,6407.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=644.50,6243.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=993.57,6136.00");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1564.77,6090.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2239.29,6249.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2742.42,6122.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3749.70,6163.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4144.52,6254.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4560.97,6025.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5622.02,6082.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=319.92,5927.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3279.86,6013.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4217.12,5827.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5050.76,5794.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5956.78,5883.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=524.09,5632.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1117.99,5584.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1754.17,5721.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2105.86,5732.40");
    }

}

